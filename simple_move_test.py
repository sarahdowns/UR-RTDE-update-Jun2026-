import numpy as np
import ur_dual_controller as urcx
import time

# --- Configuration ---
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

def main():
    """Main function to run the simple move test."""
    print("=" * 60)
    print("Starting Simple Move Test")
    print("=" * 60)
    print("This script will attempt to move the MASTER robot's first joint")
    print("by a very small amount (0.1 radians) to test basic motion.")
    print("WARNING: This script will move the robot. Press Ctrl+C to stop.")
    print("=" * 60)

    rbtx = None
    try:
        # Initialize the dual controller
        print("\n[INIT] Initializing URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        master_arm = rbtx._lft_arm

        # 1. Get the current position
        print("\n[Step 1] Reading current master arm position...")
        while not master_arm.is_running():
            print("  - Waiting for fresh data from master robot...")
            time.sleep(0.1)
        start_pose = np.array(master_arm.getj())
        print(f"✓ Current Pose: {np.round(start_pose, 4).tolist()}")

        # 2. Define a small, safe move
        target_pose = start_pose.copy()
        target_pose[0] += 0.1  # Add 0.1 radians to the base joint
        print(f"✓ Target Pose:  {np.round(target_pose, 4).tolist()}")
        
        # 3. Execute a simple 'movej' with waiting enabled.
        print("\n[Step 2] Sending simple move command...")
        print("!!! MASTER ARM WILL NOW MOVE !!!")
        
        # movej is a blocking call with its own robust waiting.
        master_arm.movej(target_pose, acc=0.5, vel=0.5, wait=True)

        print("\n✓ Simple move command completed.")
        
        # 4. Move back to start
        print("\n[Step 3] Moving back to start position...")
        master_arm.movej(start_pose, acc=0.5, vel=0.5, wait=True)
        print("\n✓ Returned to start position.")

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # Cleanly close the connections
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    main() 