import numpy as np
import ur_dual_controller as urcx
import time

class PositionCapture:
    """
    A tool to connect to the dual robot setup and capture
    the joint position of a specific robot arm.
    """
    def __init__(self, master_ip, slave_ip, pc_ip):
        """
        Initializes the controller but does not connect yet.
        """
        self.master_ip = master_ip
        self.slave_ip = slave_ip
        self.pc_ip = pc_ip
        self.rbtx = None

    def connect(self):
        """
        Initializes the connection to the robots.
        """
        print("[INFO] Initializing URDualController...")
        # Give some time for monitors to start
        time.sleep(0.5)
        self.rbtx = urcx.URDualController(
            master_robot_ip=self.master_ip,
            slave_robot_ip=self.slave_ip,
            control_pc_ip=self.pc_ip
        )
        # Give some time for the controller to be ready
        time.sleep(0.5)
        print("✓ Controller initialized successfully.")

    def get_all_positions(self):
        """
        Gets the current joint angles of both the master and slave robots.

        Returns:
            dict: A dictionary with 'master' and 'slave' numpy arrays, or None.
        """
        if not self.rbtx:
            print("[ERROR] Not connected. Please call connect() first.")
            return None

        master_arm = self.rbtx._lft_arm
        slave_arm = self.rbtx._rgt_arm

        print("[INFO] Waiting for fresh data from robots...")
        while not master_arm.is_running() or not slave_arm.is_running():
            time.sleep(0.1)
        print("✓ Both robots are sending data.")

        print("[INFO] Reading master arm joint positions...")
        master_pos = np.array(master_arm.getj())
        print("✓ Master position read.")
        
        print("[INFO] Reading slave arm joint positions...")
        slave_pos = np.array(slave_arm.getj())
        print("✓ Slave position read.")

        return {"master": master_pos, "slave": slave_pos}

    def close(self):
        """
        Closes the connection to the robots.
        """
        if self.rbtx:
            self.rbtx._lft_arm.close()
            self.rbtx._rgt_arm.close()
            print("[INFO] Robot connections closed.")

def main():
    """
    Main function to run the position capture tool.
    """
    # --- Configuration ---
    MASTER_ROBOT_IP = "192.168.5.4"
    SLAVE_ROBOT_IP = "192.168.5.5"
    CONTROL_PC_IP = "192.168.5.1"

    print("=" * 60)
    print("Robot Position Capture Tool")
    print("=" * 60)
    print("This tool will connect to the robots and print the current")
    print("joint angles of BOTH the MASTER and SLAVE arms.")
    print("=" * 60)

    capture_tool = PositionCapture(MASTER_ROBOT_IP, SLAVE_ROBOT_IP, CONTROL_PC_IP)
    try:
        capture_tool.connect()
        positions = capture_tool.get_all_positions()

        if positions is not None:
            master_position = positions["master"]
            slave_position = positions["slave"]
            
            print("\n" + "="*60)
            print("Captured Robot Positions (Joint Angles):")
            # Print in a format easy to copy into a Python list
            print(f"\nMASTER_POSE = {np.round(master_position, 4).tolist()}\n")
            print(f"SLAVE_POSE = {np.round(slave_position, 4).tolist()}\n")
            print("="*60)
            print("\nCopy the pose lines into your next script.")

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        capture_tool.close()

if __name__ == "__main__":
    main() 