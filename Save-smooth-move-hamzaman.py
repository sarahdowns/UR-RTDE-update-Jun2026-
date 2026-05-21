import numpy as np
import ur_dual_controller as urcx
import time
from trajectory import Trajectory
import argparse

# --- Configuration ---
MASTER_ROBOT_IP = "192.168.5.4"
#SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# --- Destination Poses ---
# A pose near the shoulder singularity region
DESTINATION_MASTER_POSE = [0.0, -1.57, -2.0, -0.6981, -1.57, 0.0]
#DESTINATION_SLAVE_POSE = [-0.0011, -1.9866, 1.8555, -0.9797, 1.5742, 1.6216]

# --- Home Poses ---
HOME_MASTER_POSE = [-0.0, 0.5043, -2.7961, -1.5981, -2.91, -0.0]
#HOME_SLAVE_POSE = [0.0, -3.6373, 2.8053, -1.5785, 2.6658, 0.0002]

# --- Motion Parameters ---
MOVEMENT_TIME_SECONDS = 4.0 # Time for each robot's individual move. Lower is faster.
JOINT_ACCELERATION = 1.6    # Joint acceleration.
JOINT_VELOCITY = 2.4        # Joint velocity.
CONTROL_FREQUENCY_HZ = 125  # UR robot control frequency
SINGULARITY_THRESHOLD_WRIST_RAD = 0.05 # J5 near 0
SINGULARITY_THRESHOLD_ELBOW_RAD = 0.05 # J3 near 0
# The geometric minimum distance for the wrist center to the J1 axis is d4 (0.10915m).
# We set the threshold just above this minimum possible distance.
SINGULARITY_THRESHOLD_SHOULDER_M = 0.11 # Wrist (x,y) distance from base center

def check_trajectory_for_singularities(path):
    """
    Checks a trajectory for potential singularities (wrist, elbow, shoulder).

    Args:
        path (list of lists): The list of joint configurations (waypoints).

    Returns:
        bool: True if a singularity is detected, False otherwise.
    """
    print("\n[Step 2.5] Checking trajectory for singularities...")
    
    # Standard UR5 DH parameters [d, a, alpha]
    # (theta is the joint angle variable)
    dh_params = [
        [0.089159, 0, np.pi/2],
        [0, -0.42500, 0],
        [0, -0.39225, 0],
        [0.10915, 0, np.pi/2],
        [0.09465, 0, -np.pi/2],
        [0.0823, 0, 0]
    ]

    def dh_transform_matrix(theta, d, a, alpha):
        """Computes the transformation matrix from DH parameters."""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),               np.cos(alpha),              d],
            [0,             0,                           0,                          1]
        ])

    for i, waypoint in enumerate(path):
        # 1. Wrist Singularity Check (J5 near zero)
        if abs(waypoint[4]) < SINGULARITY_THRESHOLD_WRIST_RAD:
            print(f"✗ DANGER: Wrist singularity detected in waypoint {i}!")
            print(f"  - Joint 5 angle is {np.round(np.rad2deg(waypoint[4]), 2)} degrees, which is too close to 0.")
            print("  - ABORTING MOVE.")
            return True

        # 2. Elbow Singularity Check (J3 near zero)
        if abs(waypoint[2]) < SINGULARITY_THRESHOLD_ELBOW_RAD:
            print(f"✗ DANGER: Elbow singularity detected in waypoint {i}!")
            print(f"  - Joint 3 angle is {np.round(np.rad2deg(waypoint[2]), 2)} degrees, which is too close to 0.")
            print("  - ABORTING MOVE.")
            return True
        
        # 3. Shoulder Singularity Check (wrist center over base)
        # We need to calculate the forward kinematics to the wrist center (origin of frame 5)
        T_0_1 = dh_transform_matrix(waypoint[0], dh_params[0][0], dh_params[0][1], dh_params[0][2])
        T_1_2 = dh_transform_matrix(waypoint[1], dh_params[1][0], dh_params[1][1], dh_params[1][2])
        T_2_3 = dh_transform_matrix(waypoint[2], dh_params[2][0], dh_params[2][1], dh_params[2][2])
        T_3_4 = dh_transform_matrix(waypoint[3], dh_params[3][0], dh_params[3][1], dh_params[3][2])
        T_4_5 = dh_transform_matrix(waypoint[4], dh_params[4][0], dh_params[4][1], dh_params[4][2])
        
        T_0_5 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5
        
        wrist_pos = T_0_5[:3, 3]
        wrist_dist_from_center = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
        
        if wrist_dist_from_center < SINGULARITY_THRESHOLD_SHOULDER_M:
            print(f"✗ DANGER: Shoulder singularity detected in waypoint {i}!")
            print(f"  - Wrist center is only {np.round(wrist_dist_from_center * 1000, 2)} mm from the J1 axis.")
            print("  - ABORTING MOVE.")
            return True

    print("✓ Trajectory is clear of common singularities.")
    return False

def execute_simultaneous_smooth_move(master_arm, slave_arm, master_dest, slave_dest, movement_time, acc, vel):
    """
    Moves both robot arms simultaneously from their current positions to
    their destination poses using smooth, generated trajectories.

    Args:
        master_arm: The URRobot object for the master arm.
        slave_arm: The URRobot object for the slave arm.
        master_dest (list): The target joint configuration for the master arm.
        slave_dest (list): The target joint configuration for the slave arm.
        movement_time (float): The time for the move in seconds.
        acc (float): The joint acceleration.
        vel (float): The joint velocity.
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

    # 2.5 Check for singularities
    if check_trajectory_for_singularities(master_path):
        return # Abort the move if a singularity is detected

    # 3. Execute trajectories
    print("[Step 3] Executing trajectories...")
    print("!!! MASTER ARM WILL NOW MOVE !!!")
    
    master_arm.movejs(joint_positions_list=master_path, acc=acc, vel=vel, radius=0.01, wait=False)
    # slave_arm.movejs(joint_positions_list=slave_path, acc=acc, vel=vel, radius=0.01, wait=False)

    print(f"  - Monitoring movement for up to {movement_time + 5} seconds...")
    master_arm._wait_for_move(target=master_dest, joints=True, timeout=movement_time + 5)
    # slave_arm._wait_for_move(target=slave_dest, joints=True, timeout=movement_time + 5)
    
    print("✓ Master arm movement complete.")


def main():
    """Main function to run the sequential smooth move program."""
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="Move UR robots sequentially to destination and/or home poses.")
    parser.add_argument("--check-singularity", action="store_true", help="Check a known shoulder singularity pose without moving the robot.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--home", action="store_true", help="Only move the robots to their home positions.")
    group.add_argument("--full-sequence", action="store_true", help="Move to destination, then return to home.")
    args = parser.parse_args()

    # --- Standalone Singularity Check ---
    if args.check_singularity:
        print("\n" + "="*60)
        print("--- Checking a Known Shoulder Singularity Pose ---")
        print("="*60)
        # This pose places the wrist center as close as geometrically possible to the J1 axis,
        # but keeps J5 away from 0 to avoid a simultaneous wrist singularity.
        singular_pose = [0.0, -np.pi/2, np.pi, 0.0, np.pi/4, 0.0]
        print(f"Checking Pose: {np.round(singular_pose, 4).tolist()}")
        
        # We pass a "trajectory" that is just this single point.
        check_trajectory_for_singularities(path=[singular_pose])
        print("\nCheck complete. No robot was moved.")
        print("="*60)
        return # Exit the program

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