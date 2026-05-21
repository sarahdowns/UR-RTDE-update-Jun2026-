import numpy as np
import ur_dual_controller as urcx
import time
from trajectory import Trajectory
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Configuration ---
MASTER_ROBOT_IP = "192.168.5.5"
# SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "127.0.0.1"

DESTINATION_MASTER_POSE = [0.0996, -1.3829, 0.0457, -0.2748, -4.6166, 0.0012] 

#singularity example poses:
#elbow_singularity: 0.0996, -1.3829, 0.0457, -0.2748, -4.6166, 0.0012, {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0}
#wrist_singularity: -0.0109, -1.9501, -1.8909, -3.2213, 1.2593, -0.5404, {0.0, -1.5708, -1.5708, 0.0, 3.14159, 0.0}
#shoulder_singularity: 0.7854, -1.5708, -1.5708, -1.5708, 1.5708, 0.0
#DESTINATION_SLAVE_POSE = [-0.0011, -1.9866, 1.8555, -0.9797, 1.5742, 1.6216]

# --- Motion Parameters ---
MOVEMENT_TIME_SECONDS = 4.0 # Time for each robot's individual move. 
JOINT_ACCELERATION = 1.6    # Joint acceleration.
JOINT_VELOCITY = 2.4        # Joint velocity.
CONTROL_FREQUENCY_HZ = 125  # UR robot control frequency
SINGULARITY_THRESHOLD_WRIST_RAD = 0.05 # J5 near 0
SINGULARITY_THRESHOLD_ELBOW_RAD = 0.05 # J3 near 0
SINGULARITY_THRESHOLD_SHOULDER_M = 0.01 # Wrist (x,y) distance from base center
SINGULARITY_CORRECTION_NUDGE_RAD = 0.1 # Radians to adjust joint to escape singularity

def dh_transform_matrix(theta, d, a, alpha):
    """Computes the transformation matrix from DH parameters."""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,             np.sin(alpha),               np.cos(alpha),              d],
        [0,             0,                           0,                          1]
    ])

def _calculate_forward_kinematics(waypoint, dh_params):
    """Calculates FK for a single waypoint to get wrist and flange positions."""
    T_0_1 = dh_transform_matrix(waypoint[0], dh_params[0][0], dh_params[0][1], dh_params[0][2])
    T_1_2 = dh_transform_matrix(waypoint[1], dh_params[1][0], dh_params[1][1], dh_params[1][2])
    T_2_3 = dh_transform_matrix(waypoint[2], dh_params[2][0], dh_params[2][1], dh_params[2][2])
    T_3_4 = dh_transform_matrix(waypoint[3], dh_params[3][0], dh_params[3][1], dh_params[3][2])
    T_4_5 = dh_transform_matrix(waypoint[4], dh_params[4][0], dh_params[4][1], dh_params[4][2])
    T_5_6 = dh_transform_matrix(waypoint[5], dh_params[5][0], dh_params[5][1], dh_params[5][2])
    
    T_0_5 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5
    T_0_6 = T_0_5 @ T_5_6
    
    wrist_pos = T_0_5[:3, 3]
    flange_pos = T_0_6[:3, 3]
    return wrist_pos, flange_pos

def plot_trajectories(original_path, corrected_path, dh_params):
    """Plots the original and corrected end-effector trajectories in 3D."""
    print("[Step 2.6] Plotting trajectories...")
    original_flange_path = [_calculate_forward_kinematics(wp, dh_params)[1] for wp in original_path]
    corrected_flange_path = [_calculate_forward_kinematics(wp, dh_params)[1] for wp in corrected_path]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot Original Path
    orig_x = [p[0] for p in original_flange_path]
    orig_y = [p[1] for p in original_flange_path]
    orig_z = [p[2] for p in original_flange_path]
    ax.plot(orig_x, orig_y, orig_z, 'r--', label='Original (Singular) Path')
    ax.scatter(orig_x[0], orig_y[0], orig_z[0], c='blue', s=100, marker='o', label='Start')
    ax.scatter(orig_x[-1], orig_y[-1], orig_z[-1], c='red', s=100, marker='x', label='Original End')
    
    # Plot Corrected Path
    corr_x = [p[0] for p in corrected_flange_path]
    corr_y = [p[1] for p in corrected_flange_path]
    corr_z = [p[2] for p in corrected_flange_path]
    ax.plot(corr_x, corr_y, corr_z, 'g-', label='Corrected (Safe) Path')
    ax.scatter(corr_x[-1], corr_y[-1], corr_z[-1], c='green', s=100, marker='x', label='Corrected End')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Robot Trajectory Correction')
    ax.legend()
    plt.show()

def correct_and_check_trajectory(path):
    """
    Checks a trajectory for singularities and attempts to correct them by nudging joints.
  

    Args:
        path (list of lists): The list of joint configurations (waypoints).

    Returns:
        tuple: (bool, list) where bool is True if a singularity remains,
               and list is the (potentially corrected) path.
    """
    print("\n[Step 2.5] Checking trajectory for singularities and attempting to correct...")
    corrected_path = [list(wp) for wp in path] # Make a mutable copy
    singularities_found_and_corrected = False

    # Standard UR5 DH parameters [d, a, alpha]
    dh_params = [
        [0.089159, 0, np.pi/2], [0, -0.42500, 0], [0, -0.39225, 0],
        [0.10915, 0, np.pi/2], [0.09465, 0, -np.pi/2], [0.0823, 0, 0]
    ]

    for i, waypoint in enumerate(corrected_path):
        is_waypoint_corrected = False
        # 1. Wrist Singularity Check
        if abs(waypoint[4]) < SINGULARITY_THRESHOLD_WRIST_RAD:
            print(f"  - WARNING: Wrist singularity detected in waypoint {i}. Correcting J5.")
            corrected_path[i][4] += SINGULARITY_CORRECTION_NUDGE_RAD
            singularities_found_and_corrected = True
            is_waypoint_corrected = True

        # 2. Elbow Singularity Check
        if abs(waypoint[2]) < SINGULARITY_THRESHOLD_ELBOW_RAD:
            print(f"  - WARNING: Elbow singularity detected in waypoint {i}. Correcting J3.")
            corrected_path[i][2] += SINGULARITY_CORRECTION_NUDGE_RAD
            singularities_found_and_corrected = True
            is_waypoint_corrected = True
        
        # 3. Shoulder Singularity Check
        wrist_pos, _ = _calculate_forward_kinematics(waypoint, dh_params)
        wrist_dist_from_center = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
        
        if wrist_dist_from_center < SINGULARITY_THRESHOLD_SHOULDER_M:
            print(f"  - WARNING: Shoulder singularity detected in waypoint {i}. Correcting J1.")
            # Nudging J1 rotates the base, moving the wrist away from the center
            corrected_path[i][0] += SINGULARITY_CORRECTION_NUDGE_RAD
            singularities_found_and_corrected = True
            is_waypoint_corrected = True

        if is_waypoint_corrected:
            print(f"    New corrected waypoint for index {i}: {np.round(corrected_path[i], 4).tolist()}")

    if singularities_found_and_corrected:
        print("✓ Trajectory has been corrected. Re-running checks to confirm...")
        
        # --- Re-validation Step ---
        is_still_singular = False
        for i, waypoint in enumerate(corrected_path):
            # 1. Wrist Singularity Check
            if abs(waypoint[4]) < SINGULARITY_THRESHOLD_WRIST_RAD:
                print(f"✗ FAILED: Wrist singularity still present at waypoint {i} after correction.")
                is_still_singular = True
                break
            # 2. Elbow Singularity Check
            if abs(waypoint[2]) < SINGULARITY_THRESHOLD_ELBOW_RAD:
                print(f"✗ FAILED: Elbow singularity still present at waypoint {i} after correction.")
                is_still_singular = True
                break
            # 3. Shoulder Singularity Check
            wrist_pos, _ = _calculate_forward_kinematics(waypoint, dh_params)
            wrist_dist_from_center = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
            if wrist_dist_from_center < SINGULARITY_THRESHOLD_SHOULDER_M:
                print(f"✗ FAILED: Shoulder singularity still present at waypoint {i} after correction.")
                is_still_singular = True
                break
        
        if not is_still_singular:
            print("✓ Correction successful. Path is now clear.")

        return is_still_singular, corrected_path, dh_params # Return corrected path and status
    else:
        print("✓ Trajectory is clear of common singularities.")
        return False, path, dh_params # Return original path

def execute_simultaneous_smooth_move(master_arm, slave_arm, master_dest, slave_dest, movement_time, acc, vel):
    """
    Moves both robot arms simultaneously from their current positions to
    their destination poses using smooth, generated trajectories.
    """
    print("\n--- Moving Both Arms Simultaneously ---")

    # 1. Get current positions
    print("[Step 1] Reading current arm positions...")
    master_start_pose = np.array(master_arm.getj())
    # slave_start_pose = np.array(slave_arm.getj())
    print(f"✓ Master Start Pose: {np.round(master_start_pose, 4).tolist()}")
    # print(f"✓ Slave Start Pose:  {np.round(slave_start_pose, 4).tolist()}")

    # 2. Generate trajectories
    print("[Step 2] Generating smooth trajectories...")
    traj_generator = Trajectory(method="quintic")
    control_frequency = 1.0 / CONTROL_FREQUENCY_HZ
    
    master_path, _ = traj_generator.piecewise_interpolation(
        path=[master_start_pose, master_dest],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    # slave_path, _ = traj_generator.piecewise_interpolation(
    #     path=[slave_start_pose, slave_dest],
    #     control_frequency=control_frequency,
    #     interval_time=movement_time
    # )
    print(f"✓ Trajectories generated with {len(master_path)} waypoints.")

    # 2.5 Check for singularities and correct if possible
    is_still_singular, corrected_master_path, dh_params = correct_and_check_trajectory(master_path)

    if is_still_singular:
        print("✗ FAILED: Could not correct a singularity in the path. ABORTING MOVE.")
        return # Abort the move if a singularity is detected and could not be fixed
    
    # Plot if a correction was made
    if corrected_master_path is not master_path:
        print("\n" + "="*60)
        print("ACTION REQUIRED: Singularity Corrected")
        print("The planned trajectory was adjusted to avoid a singularity.")
        print("A plot showing the original vs. corrected path will be displayed.")
        print(">>> PLEASE CLOSE THE PLOT WINDOW TO START THE ROBOT'S MOVEMENT. <<<")
        print("="*60 + "\n")
        plot_trajectories(master_path, corrected_master_path, dh_params)

    # 3. Execute trajectories
    print("[Step 3] Executing trajectories...")
    print("!!! MASTER ARM WILL NOW MOVE !!!")
    
    master_arm.movejs(joint_positions_list=corrected_master_path, acc=acc, vel=vel, radius=0.01, wait=False)
    # slave_arm.movejs(joint_positions_list=slave_path, acc=acc, vel=vel, radius=0.01, wait=False)

    print(f"  - Monitoring movement for up to {movement_time + 5} seconds...")
    # Note: The target for _wait_for_move should be the final waypoint of the executed path
    final_master_dest = corrected_master_path[-1]
    master_arm._wait_for_move(target=final_master_dest, joints=True, timeout=movement_time + 5)
    # slave_arm._wait_for_move(target=slave_dest, joints=True, timeout=movement_time + 5)
    
    print("✓ Master arm movement complete.")


def main():
    """Main function to run the sequential smooth move program."""
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="Move UR robots sequentially to destination and/or home poses.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--home", action="store_true", help="Only move the robots to their home positions.")
    group.add_argument("--full-sequence", action="store_true", help="Move to destination, then return to home.")
    args = parser.parse_args()

    print("=" * 60)
    print("Starting Sequential Smooth Move Program")
    print("=" * 60)
    print("WARNING: This script will move the robots. Press Ctrl+C to stop.")
    print("=" * 60)

    rbtx = None
    try:
        # Initialize the dual controller
        print("\n[INIT] Initializing URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            # slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        master_arm = rbtx._lft_arm
        # slave_arm = rbtx._rgt_arm
        slave_arm = None # Assign None to prevent errors

        if args.home:
            # --- Go directly to Home ---
            print("\n" + "="*60)
            print("  MOVING DIRECTLY TO HOME POSITIONS")
            print("="*60 + "\n")
            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                HOME_MASTER_POSE, None,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

        elif args.full_sequence:
            # --- Full Sequence: Destination then Home ---
            print("\n" + "="*60)
            print("Part 1: Moving to Destination")
            print(f"Target Master Pose: {np.round(DESTINATION_MASTER_POSE, 4).tolist()}")
            # print(f"Target Slave Pose:  {np.round(DESTINATION_SLAVE_POSE, 4).tolist()}")
            print("="*60)

            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                DESTINATION_MASTER_POSE, None,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

            # --- Return to Home ---
            print("\n" + "="*60)
            print("Part 2: RETURNING TO HOME POSITIONS")
            print("="*60 + "\n")

            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                HOME_MASTER_POSE, None,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            
        else:
            # --- Destination Only ---
            print("\n" + "="*60)
            print("Moving to Destination Poses")
            print(f"Target Master Pose: {np.round(DESTINATION_MASTER_POSE, 4).tolist()}")
            # print(f"Target Slave Pose:  {np.round(DESTINATION_SLAVE_POSE, 4).tolist()}")
            print("="*60)

            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                DESTINATION_MASTER_POSE, None,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # Cleanly close the connections
        if rbtx:
            rbtx._lft_arm.close()
            # rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    main() 
