import math
import numpy as np
import ur_dual_controller as urcx

# --- Configuration ---
# Your confirmed working IP addresses
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# --- Speed Control ---
# The time in seconds for each segment of the wave.
# Larger value = SLOWER movement.
MOVEMENT_SPEED_SECONDS = 1.0

# --- Wave Definition ---
# The joint to use for waving (joint 3 is the upper arm/elbow joint)
WAVE_JOINT_INDEX = 3
# How far the joint will move in radians (0.5 radians is about 28 degrees)
WAVE_ANGLE_DELTA = 0.5
# How many times to wave back and forth
WAVE_REPETITIONS = 2


def generate_wave_path(initial_jnt_values):
    """
    Generates a list of joint positions for a 'wave' motion for one arm.
    """
    path = []
    wave_point_1 = initial_jnt_values.copy()
    wave_point_1[WAVE_JOINT_INDEX] -= WAVE_ANGLE_DELTA
    wave_point_2 = initial_jnt_values.copy()
    wave_point_2[WAVE_JOINT_INDEX] += WAVE_ANGLE_DELTA

    # Assemble the path: Start -> P1 -> P2 -> P1 -> ... -> Start
    path.append(initial_jnt_values.tolist())
    for _ in range(WAVE_REPETITIONS):
        path.append(wave_point_1.tolist())
        path.append(wave_point_2.tolist())
    path.append(initial_jnt_values.tolist())
    
    return path


def main():
    """Main function to run the synchronized waving program."""
    print("=" * 60)
    print("Starting Synchronized Dual Arm Wave Program")
    print("=" * 60)
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

        # 2. Get the current joint positions of both arms
        print("\n[Step 2] Reading initial joint positions...")
        master_start_pos = np.array(rbtx._lft_arm.getj())
        slave_start_pos = np.array(rbtx._rgt_arm.getj())
        print(f"✓ Master Arm: {np.round(master_start_pos, 2)}")
        print(f"✓ Slave Arm:  {np.round(slave_start_pos, 2)}")

        # 3. Generate a wave path for each arm independently
        print("\n[Step 3] Generating wave trajectories for each arm...")
        path_master = generate_wave_path(master_start_pos)
        path_slave = generate_wave_path(slave_start_pos)
        print("✓ Trajectories generated.")

        # 4. Combine the paths for the dual controller
        # The controller expects a list of 12-element joint positions.
        print("\n[Step 4] Combining trajectories for synchronized movement...")
        combined_path = [master_joints + slave_joints for master_joints, slave_joints in zip(path_master, path_slave)]
        print(f"✓ Combined path created with {len(combined_path)} waypoints.")

        # 5. Execute the synchronized movement
        print("\n[Step 5] Sending trajectory to robots...")
        print("!!! ROBOTS WILL NOW WAVE SIMULTANEOUSLY !!!")
        rbtx.move_jntspace_path(combined_path, control_frequency=0.05, interval_time=MOVEMENT_SPEED_SECONDS)
        print("\n✓ Movement command sent and executed.")

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # 6. Cleanly close the connections
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    main() 