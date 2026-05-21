import numpy as np
import ur_dual_controller as urcx
import time
from trajectory import Trajectory
import argparse

# --- Configuration ---
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "127.0.0.1"

# --- Destination Poses ---
# Using a standard, safe "home" pose for both arms to ensure success.
DESTINATION_MASTER_POSE = [-0.0067, -1.207, -1.7849, 0.3803, -4.5978, 1.6513]
DESTINATION_SLAVE_POSE = [-0.0011, -1.9866, 1.8555, -0.9797, 1.5742, 1.6216]

# --- Home Poses ---
HOME_MASTER_POSE = [-0.0, 0.5043, -2.7961, -1.5981, -2.91, -0.0]
HOME_SLAVE_POSE = [0.0, -3.6373, 2.8053, -1.5785, 2.6658, 0.0002]

# --- Motion Parameters ---
MOVEMENT_TIME_SECONDS = 4.0 # Time for each robot's individual move. Lower is faster.
JOINT_ACCELERATION = 1.6    # Joint acceleration.
JOINT_VELOCITY = 2.4        # Joint velocity.
CONTROL_FREQUENCY_HZ = 125  # UR robot control frequency

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
    slave_start_pose = np.array(slave_arm.getj())
    print(f"✓ Master Start Pose: {np.round(master_start_pose, 4).tolist()}")
    print(f"✓ Slave Start Pose:  {np.round(slave_start_pose, 4).tolist()}")

    # 2. Generate trajectories
    print("[Step 2] Generating smooth trajectories...")
    # A trajectory is a sequence of points that the robot will follow.
    #  a quintic (5th-order polynomial) interpolation for a smooth path.
    traj_generator = Trajectory(method="quintic")
    control_frequency = 1.0 / CONTROL_FREQUENCY_HZ
    
    # Generate a path for the master arm from its start to destination pose.
    master_path, _ = traj_generator.piecewise_interpolation(
        path=[master_start_pose, master_dest],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    # Generate a path for the slave arm from its start to destination pose.
    slave_path, _ = traj_generator.piecewise_interpolation(
        path=[slave_start_pose, slave_dest],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    print(f"✓ Trajectories generated with {len(master_path)} waypoints.")

    # 3. Execute trajectories
    print("[Step 3] Executing trajectories...")
    print("!!! BOTH ARMS WILL NOW MOVE !!!")
    
    # Send the entire trajectory to each robot's controller.
    # `wait=False` is crucial: it allows both commands to be sent without blocking,
    # enabling the arms to move at the same time.
    master_arm.movejs(joint_positions_list=master_path, acc=acc, vel=vel, radius=0.01, wait=False)
    slave_arm.movejs(joint_positions_list=slave_path, acc=acc, vel=vel, radius=0.01, wait=False)

    print(f"  - Monitoring movement for up to {movement_time + 5} seconds...")
    # Although the move commands are non-blocking, we need to wait for them to finish
    # before proceeding. This function monitors the robot's status.
    master_arm._wait_for_move(target=master_dest, joints=True, timeout=movement_time + 5)
    slave_arm._wait_for_move(target=slave_dest, joints=True, timeout=movement_time + 5)
    
    print("✓ Simultaneous arm movement complete.")


def main():
    """Main function to run the sequential smooth move program."""
    # --- Argument Parsing ---
    # This section allows running the script with different behaviors from the command line,
    # e.g., `python sequential_smooth_move.py --home` to only move to the home position.
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
    # The main logic is wrapped in a try...finally block to ensure that
    # robot connections are closed cleanly, even if an error occurs.
    try:
        # Initialize the dual controller
        print("\n[INIT] Initializing URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        # For convenience, assign the left and right arms to separate variables.
        master_arm = rbtx._lft_arm
        slave_arm = rbtx._rgt_arm

        if args.home:
            # --- Go directly to Home ---
            print("\n" + "="*60)
            print("  MOVING DIRECTLY TO HOME POSITIONS")
            print("="*60 + "\n")
            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                HOME_MASTER_POSE, HOME_SLAVE_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

        elif args.full_sequence:
            # --- Full Sequence: Destination then Home ---
            print("\n" + "="*60)
            print("Part 1: Moving to Destination")
            print(f"Target Master Pose: {np.round(DESTINATION_MASTER_POSE, 4).tolist()}")
            print(f"Target Slave Pose:  {np.round(DESTINATION_SLAVE_POSE, 4).tolist()}")
            print("="*60)

            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                DESTINATION_MASTER_POSE, DESTINATION_SLAVE_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

            # --- Return to Home ---
            print("\n" + "="*60)
            print("Part 2: RETURNING TO HOME POSITIONS")
            print("="*60 + "\n")

            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                HOME_MASTER_POSE, HOME_SLAVE_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            
        else:
            # --- Destination Only ---
            print("\n" + "="*60)
            print("Moving to Destination Poses")
            print(f"Target Master Pose: {np.round(DESTINATION_MASTER_POSE, 4).tolist()}")
            print(f"Target Slave Pose:  {np.round(DESTINATION_SLAVE_POSE, 4).tolist()}")
            print("="*60)

            execute_simultaneous_smooth_move(
                master_arm, slave_arm,
                DESTINATION_MASTER_POSE, DESTINATION_SLAVE_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # This block will always execute, ensuring a clean shutdown.
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    main() 
