import numpy as np
import ur_dual_controller as urcx
import time
from trajectory import Trajectory

# --- Configuration ---
# Your confirmed working IP addresses
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# The destination pose you captured for the slave robot.
DESTINATION_POSE = [-1.6675, -1.509, 0.0841, -3.2966, -0.8302, 0.0008]

# --- Motion Parameters ---
# Total time in seconds for the movement from start to destination.
# A larger value will result in a slower, more deliberate motion.
MOVEMENT_TIME_SECONDS = 8.0

# The control frequency for sending commands to the robot.
# This should generally not be changed.
CONTROL_FREQUENCY_HZ = 125 # UR robots internal control loop frequency


def main():
    """Main function to run the advanced wave program."""
    print("=" * 60)
    print("Starting Advanced Wave Program")
    print("=" * 60)
    print("This script will move the SLAVE robot from its current")
    print("position to your saved destination pose smoothly.")
    print("WARNING: This script will move the robots. Press Ctrl+C to stop.")
    print("=" * 60)

    rbtx = None
    try:
        # 1. Initialize the dual controller
        print("\n[Step 1] Initializing URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        # Get the individual slave arm object
        slave_arm = rbtx._rgt_arm

        # 2. Get the current position of the slave arm
        print("\n[Step 2] Reading current slave arm position...")
        while not slave_arm.is_running():
            print("  - Waiting for fresh data from slave robot...")
            time.sleep(0.1)
        start_pose = np.array(slave_arm.getj())
        print(f"✓ Current Pose: {np.round(start_pose, 4).tolist()}")
        print(f"✓ Target Pose:  {DESTINATION_POSE}")

        # 3. Generate a smooth trajectory
        print("\n[Step 3] Generating smooth trajectory...")
        # Use the quintic polynomial interpolator for a smoother result
        traj_generator = Trajectory(method="quintic")
        
        # The path is just two points: start and destination
        path_to_generate = [start_pose, DESTINATION_POSE]
        
        # The control frequency is how often a command is sent. 
        # For this library, it's 1/125 = 0.008s
        control_frequency = 1.0 / CONTROL_FREQUENCY_HZ

        # The piecewise_interpolation method generates the path.
        # It returns both positions and speeds; we only need positions.
        path_positions, _ = traj_generator.piecewise_interpolation(
            path=path_to_generate,
            control_frequency=control_frequency,
            interval_time=MOVEMENT_TIME_SECONDS
        )
        
        print(f"✓ Trajectory generated with {len(path_positions)} waypoints.")

        # 4. Execute the trajectory
        print("\n[Step 4] Executing trajectory...")
        print("!!! SLAVE ARM WILL NOW MOVE !!!")
        
        # The correct method to execute a list of joint positions is 'movejs'.
        # We set 'wait=False' because this is a long trajectory and the
        # default timeout in the library is too short.
        slave_arm.movejs(joint_positions_list=path_positions, acc=0.8, vel=1.2, radius=0.01, wait=False)

        # We know the movement takes MOVEMENT_TIME_SECONDS, so we wait here.
        print(f"  - Waiting for {MOVEMENT_TIME_SECONDS} seconds for trajectory to complete...")
        time.sleep(MOVEMENT_TIME_SECONDS + 1) # Add a small buffer

        print("\n✓ Movement complete.")

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # 5. Cleanly close the connections
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    main() 