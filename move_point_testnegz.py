# File name: test_move_setpoint.py

import time
import numpy as np
from move_xyz_safe import UR5eSafeController

# System Settings
ip = "192.168.5.5"
TOUCH_FORCE_THRESHOLD = 7.0   # Stop immediately if force changes by more than 7 Newtons
DESCENT_SPEED = -0.03          # Move downward at a very slow, safe 1 cm/s
MAX_DESCENT_DISTANCE = 0.30    # Maximum distance (meters) to search before aborting

def main():
    print("[INIT] Connecting to UR5e Safe Controller...")
    robot = UR5eSafeController(ip=ip)
    
    try:
        # 1. Establish a safe starting position above the suspected table height
        # Replace these coordinates with a known safe hover point in space
        hover_x, hover_y, hover_z = 0.3, -0.5, 0.15
        print(f"[STAGE 1] Moving to safe baseline hover height: Z = {hover_z}m")
        robot.move_to_xyz_safe(hover_x, hover_y, hover_z, visualize=False, speed=0.1)
        time.sleep(1.0) # Wait for mechanical oscillations to settle completely
        
        # 2. Tare the sensor while stationary to clear tool weight gravity bias
        robot.zero_ft_sensor()
        time.sleep(0.5)
        
        # Capture baseline forces
        baseline_ft = robot.get_ft_sensor_data()
        baseline_f_z = baseline_ft[2]
        print(f"[STAGE 2] F/T Sensor tared. Baseline Z Force: {baseline_f_z:.2f} N")
        
        # Track initial starting height to enforce search window safety
        start_pose = robot.rtde_r.getActualTCPPose()
        start_z = start_pose[2]
        
        print(f"[STAGE 3] Beginning slow touch descent at {abs(DESCENT_SPEED)*100:.1f} cm/s...")
        
        table_detected = False
        recorded_table_z = None
        
        # 3. Execution Control Loop
        V_world = np.array([0.0, 0.0, DESCENT_SPEED])
        
        # Define the base-to-world rotation matrix (+45 deg on Y-axis)
        theta = np.deg2rad(45)
        R_world_base = np.array([
            [np.cos(theta),  0.0, np.sin(theta)],
            [0.0,            1.0, 0.0          ],
            [-np.sin(theta), 0.0, np.cos(theta)]
        ])
        
        # Rotate the World velocity vector into the Robot's raw Base frame
        V_base = R_world_base.T @ V_world 
        
        # Construct the 6D spatial velocity command [Vx, Vy, Vz, Rx, Ry, Rz]
        speedl_command = [V_base[0], V_base[1], V_base[2], 0.0, 0.0, 0.0]

        while True:
            current_pose = robot.rtde_r.getActualTCPPose()
            current_z = current_pose[2]
            
            # Distance traveled safety check
            if abs(start_z - current_z) > MAX_DESCENT_DISTANCE:
                print(f"[ABORT] Traveled max distance ({MAX_DESCENT_DISTANCE}m) without contact.")
                break
                
            # Query real-time force data
            current_ft = robot.get_ft_sensor_data()
            current_f_z = current_ft[2]
            
            # Calculate absolute impact force deflection (Isolating contact)
            delta_force_z = abs(current_f_z - baseline_f_z)
            
            # Check if threshold has been tripped
            if delta_force_z > TOUCH_FORCE_THRESHOLD:
                # INSTANT ACTION: Command immediate physical stop
                robot.rtde_c.stopL(2.0) 
                
                # Capture the precise contact coordinate
                recorded_table_z = current_z
                table_detected = True
                
                print("\n" + "="*40)
                print("TABLE TOP CONTACT DETECTED!")
                print("="*40)
                print(f"Delta Contact Force: {delta_force_z:.2f} N")
                print(f"Registered Table Top Z Coordinate: {recorded_table_z:.4f} m")
                print("="*40 + "\n")
                break
            
            # Send the rotated asynchronous linear velocity vector command
            # Format: [Vx, Vy, Vz, Rx, Ry, Rz] -> time=0.02 keeps it alive for 20ms
            robot.rtde_c.speedL(speedl_command, acceleration=0.2, time=0.02)
            
            # Tiny sleep to match UR control cycle (500Hz for e-Series)
            time.sleep(0.002)
            
    except Exception as e:
        print(f"[CRASH] Loop error encountered: {e}")
    finally:
        # Clean shutdown sequence ensures velocity threads clear
        print("[CLEANUP] Stopping arm and closing channels...")
        if 'robot' in locals():
            robot.rtde_c.speedStop()
            robot.cleanup()
        print("[STATUS] Shutdown complete.")

if __name__ == "__main__":
    main()
