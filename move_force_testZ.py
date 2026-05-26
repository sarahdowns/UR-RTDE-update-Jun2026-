# Author: Sarah Downs
# File name: move_force_testZ
# This code moves the robot to an xyz location then lowers in robot base z until contact, then 
# loosens gripper and moves back up in z.

import time
import numpy as np
from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2

# System Settings
ip = "192.168.5.5"
rg_id = 0
TOUCH_FORCE_THRESHOLD = 7.0    # Stop immediately if force changes by more than 7 Newtons
DESCENT_SPEED = -0.03          # Move downward at a very slow, safe 3 cm/s
MAX_DESCENT_DISTANCE = 0.30    # Maximum distance (meters) to search before aborting
ROLL_DEG = 90    # +90 rotates counter-clockwise around the X-axis
PITCH_DEG = 180  # Keep baseline pitch pointing down (adjust if needed)
YAW_DEG = 0

gripper_open = 45
gripper_closed = 20
gripper_force = 25

def main():
    print("[INFO] Connecting to UR5e Safe Controller...")
    robot = UR5eSafeController(ip=ip)
    
    print("[INFO] Connecting to OnRobot RG Gripper...")
    rg_gripper = RG2(ip, rg_id)
    
    print("Gripper closed.")
    rg_gripper.rg_grip(gripper_closed, gripper_force)
    time.sleep(0.5)
    
    try:
        # 1. Establish a safe starting position above the suspected table height
        hover_x, hover_y, hover_z = 0.3, -0.68, 0.2
        print(f"[STAGE 1] Moving to safe baseline hover height: Z = {hover_z}m")
        robot.move_to_xyz_safe(hover_x, hover_y, hover_z, visualize=False, speed=0.1, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG)
        time.sleep(1.0) # Wait for mechanical oscillations to settle completely
        
        # 2. Tare the sensor while stationary to clear tool weight gravity bias
        robot.zero_ft_sensor()
        time.sleep(0.5)
        
        # Capture baseline forces using your world-rotated function
        baseline_ft = robot.get_ft_sensor_baserot(world_frame=True)
        baseline_f_z = baseline_ft[2]
        print(f"[STAGE 2] F/T Sensor tared. Baseline World Z Force: {baseline_f_z:.2f} N")
        
        print(f"[STAGE 3] Beginning slow touch descent at {abs(DESCENT_SPEED)*100:.1f} cm/s...")
        
        # Define the base-to-world rotation matrix (+45 deg on Y-axis) for the velocity vector
        theta = np.deg2rad(45)
        R_world_base = np.array([
            [np.cos(theta),  0.0, np.sin(theta)],
            [0.0,            1.0, 0.0          ],
            [-np.sin(theta), 0.0, np.cos(theta)]
        ])
        
        # Rotate the World velocity vector into the Robot's raw Base frame
        V_world = np.array([0.0, 0.0, DESCENT_SPEED])
        V_base = R_world_base.T @ V_world 
        speedl_command = [V_base[0], V_base[1], V_base[2], 0.0, 0.0, 0.0]
        
        table_detected = False
        recorded_world_pose = None
        
        # 3. Execution Control Loop
        while True:
            # Track exact Base Frame pose directly from the UR Controller
            current_pose = robot.rtde_r.getActualTCPPose()
            current_z = current_pose[2]
            
            # Distance traveled safety check
            if abs(hover_z - current_z) > MAX_DESCENT_DISTANCE: # (Note: hover_z and current_z comparison is an approximation here)
                print(f"[ABORT] Traveled max distance without contact.")
                break
                
            # Query real-time force data, rotated to the World Frame
            current_ft = robot.get_ft_sensor_baserot(world_frame=True)
            current_f_z = current_ft[2]
            
            # Calculate absolute impact force deflection against the table
            delta_force_z = abs(current_f_z - baseline_f_z)
            
            # Check if threshold has been tripped
            if delta_force_z > TOUCH_FORCE_THRESHOLD:
                # INSTANT ACTION: Terminate the velocity thread properly
                robot.rtde_c.speedStop() 
                
                # Capture the precise contact coordinate in the raw Base Frame
                recorded_base_pose = current_pose
                table_detected = True
                
                print("\n" + "="*40)
                print("TABLE TOP CONTACT DETECTED!")
                print("="*40)
                print(f"Delta Contact Force: {delta_force_z:.2f} N")
                print("="*40 + "\n")
                break
            
            # Send the rotated asynchronous linear velocity vector command
            robot.rtde_c.speedL(speedl_command, acceleration=0.2, time=0.02)
            time.sleep(0.002)
            
        # 4. Post-Touch Action (Release and Retract)
        if table_detected and recorded_base_pose is not None:
            # Step A: Release the Gripper
            print("[ACTION] Releasing payload...")
            rg_gripper.rg_grip(60, 25.0)
            time.sleep(1.0) # Allow physical jaws to open completely
            
            # Step B: Retract 2cm straight up in the World Frame (using Native Controller Math)
            print("[SAFE] Retracting 10cm in +Z (World Frame)...")
            
            # We want to move +0.02m in the World Z. Rotate this vector back to the Base frame.
            Delta_World = np.array([0.0, 0.0, 0.1])
            Delta_Base = R_world_base.T @ Delta_World
            
            # Copy the exact recorded pose and apply the 2cm spatial shift
            target_pose = list(recorded_base_pose)
            target_pose[0] += Delta_Base[0]
            target_pose[1] += Delta_Base[1]
            target_pose[2] += Delta_Base[2]
            
            # Use UR's native linear move: rtde_c.moveL([x, y, z, rx, ry, rz], speed, accel)
            # This perfectly holds the exact orientation it had upon impact
            robot.rtde_c.moveL(target_pose, 0.05, 0.2)
            print("[STATUS] Sequence complete.")
            
    except Exception as e:
        print(f"[CRASH] Loop error encountered: {e}")
    finally:
        print("[CLEANUP] Stopping arm and closing channels...")
        if 'robot' in locals() and robot is not None:
            robot.rtde_c.speedStop()
            robot.cleanup()
        print("[STATUS] Shutdown complete.")

if __name__ == "__main__":
    main()
