import math
import numpy as np
import ur_dual_controller as urcx
import time

# --- Configuration ---
# Your confirmed working IP addresses
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# --- Speed Control ---
# Higher values = FASTER movement.
WAVE_ACCELERATION = 2.0
WAVE_VELOCITY = 1.5

# --- Wave Definition ---
# The joint to use for waving (e.g., joint 4 is wrist 2)
WAVE_JOINT_INDEX = 2

# How far the joint will move in radians (0.5 radians is about 28 degrees)
WAVE_ANGLE_DELTA = 0.5
# How many times to wave back and forth
WAVE_REPETITIONS = 1


def generate_wave_path(initial_jnt_values):
    """
    Generates a list of joint positions for a 'wave' motion.

    A wave is defined as moving a single joint back and forth from its
    starting position.

    Args:
        initial_jnt_values (np.array): The starting joint configuration.

    Returns:
        list: A list of joint configurations forming the wave path.
    """
    path = []
    
    # Create the two extreme points of the wave
    wave_point_1 = initial_jnt_values.copy()
    wave_point_1[WAVE_JOINT_INDEX] -= WAVE_ANGLE_DELTA

    wave_point_2 = initial_jnt_values.copy()
    wave_point_2[WAVE_JOINT_INDEX] += WAVE_ANGLE_DELTA

    # Assemble the path: Start -> P1 -> P2 -> P1 -> ... -> Start
    path.append(initial_jnt_values.tolist()) # Ensure we start from the initial pose
    for _ in range(WAVE_REPETITIONS):
        path.append(wave_point_1.tolist())
        path.append(wave_point_2.tolist())
    
    # Return to the starting position smoothly
    path.append(initial_jnt_values.tolist())
    
    return path


def execute_wave(robot_arm, arm_name):
    """
    Gets the robot's current position and executes a wave motion.

    Args:
        robot_arm: The URRobot object for the arm to wave.
        arm_name (str): The name of the arm for printing messages.
    """
    print(f"\n--- Waving {arm_name} Arm ---")
    
    # 1. Get the arm's current position to start the wave from there.
    print(f"[Step 1] Reading initial {arm_name} arm joint positions...")
    initial_joints = np.array(robot_arm.getj())
    print(f"✓ {arm_name} Arm initial positions: {np.round(initial_joints, 2)}")

    # 2. Generate the wave trajectory based on this position.
    print("[Step 2] Generating wave trajectory...")
    wave_path = generate_wave_path(initial_joints)
    print("✓ Trajectory generated.")

    # 3. Execute the move for each point in the path.
    # We use movej, which is a blocking call. The script will wait for
    # each movement to finish before sending the next one.
    print(f"!!! {arm_name} ARM WILL NOW WAVE !!!")
    for i, target_joints in enumerate(wave_path):
        print(f"  -> Moving to waypoint {i+1}/{len(wave_path)}...")
        robot_arm.movej(target_joints, acc=WAVE_ACCELERATION, vel=WAVE_VELOCITY, wait=True)
    print(f"✓ {arm_name} arm wave complete.")


def main():
    """Main function to run the waving program."""
    print("=" * 60)
    print("Starting Sequential Dual Arm Wave Program")
    print("=" * 60)
    print("WARNING: This script will move the robots. Press Ctrl+C to stop.")
    print("Ensure the area around the robots is clear.")
    print("=" * 60)

    rbtx = None  # Initialize to None for the finally block
    try:
        # 1. Initialize the dual controller
        print("\n[INIT] Initializing URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        # From the dual controller, we can get the individual robot objects.
        master_arm = rbtx._lft_arm
        slave_arm = rbtx._rgt_arm

        # 2. Execute the wave on the Master Arm
        execute_wave(master_arm, "Master")

        # 3. Wait for 5 seconds
        # print("\n--- Waiting for 5 seconds ---")
        # time.sleep(5)

        # 4. Execute the wave on the Slave Arm
        execute_wave(slave_arm, "Slave")

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