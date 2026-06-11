# Author: Sarah Downs
# File name: move_MC_stiff.py
# This code uses a Monte Carlo distribution from camera data to test inserting a rod into a socket automatically.

import os
import glob
import numpy as np
import pandas as pd
import time
from spatialmath import SE3
from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2

# System Configuration
ip = "192.168.5.5"
rg_id = 0
CALIB_FILE = "cam_to_robot_transform.npy"

# Testing Parameters
NUM_TESTS = 60          
HOVER_OFFSET = 0.15     
SOCKET_SURFACE_Z = 0.11 
INSERTION_DEPTH = 0.13
ROD_PROTRUSION = 0.13
FORCE_THRESHOLD = 20.0   
GRIPPER_FORCE = 0
GRIPPER_WIDTH = 21.0    # Target width in mm for the jaws to close around the rod

rg_gripper = RG2(ip, rg_id)

pitch_deg = 0
roll_deg = -90
yaw_deg = 90

def load_latest_csv(folder='.'):
    """Finds the most recently generated camera data file."""
    csv_files = glob.glob(os.path.join(folder, "tb_camxyz_*.csv"))
    if not csv_files:
        raise FileNotFoundError("No camera data CSV found. Run your capture script first!")
    return max(csv_files, key=os.path.getctime)

def main():
    # 1. Load Data and Calculate Distribution
    data_file = load_latest_csv()
    print(f"[INFO] Loading vision data from: {data_file}")
    
    df = pd.read_csv(data_file)
    mu = df[['Ball_X', 'Ball_Y', 'Ball_Z']].mean().values
    std = df[['Ball_X', 'Ball_Y', 'Ball_Z']].std().values
    
    print(f"[STATISTICS] Data Mean: X={mu[0]:.4f}, Y={mu[1]:.4f}, Z={mu[2]:.4f}")
    print(f"[STATISTICS] Data Std:  X={std[0]:.5f}, Y={std[1]:.5f}, Z={std[2]:.5f}")

    if not os.path.exists(CALIB_FILE):
        print(f"[ERROR] Calibration matrix {CALIB_FILE} not found.")
        return
        
    T_cam_to_robot = np.load(CALIB_FILE)
    T_world_base = SE3.Ry(np.deg2rad(45))

    print("\n[INIT] Connecting to UR5e...")
    robot = UR5eSafeController(ip)

    # Statistical Counters
    successes = 0
    failures = 0

    try:
        user_input = input(f"\n[PROMPT] Ready to begin {NUM_TESTS} automated insertions? (y/n): ")
        if user_input.lower() != 'y':
            print("[INFO] Aborting.")
            return

        # --- ENGAGE GRIPPER ---
        print(f"[ACTION] Engaging gripper at {GRIPPER_FORCE} N...")
        rg_gripper.rg_grip(GRIPPER_WIDTH, GRIPPER_FORCE)
        time.sleep(1.0) # Give the physical jaws time to clamp the rod securely

        for i in range(1, NUM_TESTS + 1):
            print(f"\n========================================")
            print(f"       TEST {i} OF {NUM_TESTS}")
            print(f"========================================")

            # --- A. Draw Random Coordinate ---
            p_cam = np.random.normal(loc=mu, scale=std)
            p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
            p_base = T_cam_to_robot @ p_cam_h
            p_world = T_world_base.A @ np.array([p_base[0], p_base[1], p_base[2], 1.0])
            
            target_x = p_world[0]
            target_y = p_world[1]
            target_z = SOCKET_SURFACE_Z 
            
            print(f"[MONTE CARLO] Target: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")

            # --- B. Move to Safe Hover ---
            # ask_user=False bypasses the prompt for total automation
            robot.move_to_xyz_safe(target_x, target_y, target_z + HOVER_OFFSET, 
                                   visualize=False, roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg,
                                   ask_user=False, async_move=False)
            
            # --- C. Guarded Insertion with High-Speed Logging ---
            print(f"[ACTION] Lowering rod into socket...")
            robot.zero_ft_sensor() 
            
            # Initialize an empty list for THIS specific insertion attempt
            current_insertion_data = []
            
            # Start moving down asynchronously
            robot.move_to_xyz_safe(target_x, target_y, (target_z - INSERTION_DEPTH) + ROD_PROTRUSION, 
                                   visualize=False, roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg,
                                   ask_user=False, async_move=True)
            
            jammed = False
            time.sleep(0.5) # Allow arm to start moving
            
            start_time = time.time() # Start the stopwatch for our plot's X-axis

            while not robot.rtde_c.isSteady():
                current_time = time.time() - start_time
                
                # 1. Read Force (Rotated to World Frame)
                ft_data = robot.get_ft_sensor_baserot(world_frame=True)
                z_force = ft_data[2]  
                
                # 2. Read End Effector Position (Raw Base Frame)
                current_pose = robot.rtde_r.getActualTCPPose() 
                
                # Convert Base Frame Position to World Frame
                p_base_h = np.array([current_pose[0], current_pose[1], current_pose[2], 1.0])
                p_world_h = T_world_base.A @ p_base_h
                
                # Calculate the exact location of the ROD TIP in the World Frame
                tip_world_x = p_world_h[0]
                tip_world_y = p_world_h[1]
                tip_world_z = p_world_h[2] - ROD_PROTRUSION 
                
                # 3. Log to RAM instantly (Storing Tip World Coordinates!)
                current_insertion_data.append([
                    i,              
                    current_time,   
                    tip_world_x, tip_world_y, tip_world_z,             # Tip X, Y, Z (World)
                    ft_data[0], ft_data[1], ft_data[2],                # Force X, Y, Z (World)
                    target_x, target_y                                 
                ])
                
                # 4. Safety Guard
                if abs(z_force) > FORCE_THRESHOLD:
                    print(f"[DANGER] Force spike ({abs(z_force):.2f} N). Socket missed or jammed!")
                    robot.rtde_c.stopJ(2.0)  # Brake instantly
                    jammed = True
                    break
                    
                time.sleep(0.01) # 100Hz logging rate

            # --- D. Retract and Save Data ---
            if jammed:
                failures += 1
                status = "FAILED"
                print(f"[ACTION] Retracting safely to hover...")
            else:
                successes += 1
                status = "SUCCESS"
                time.sleep(0.5)
                print(f"[SUCCESS] Clean insertion! Retracting...")

            # Pull back up to hover
            robot.move_to_xyz_safe(target_x, target_y, target_z + HOVER_OFFSET, 
                                   visualize=False, roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg,
                                   ask_user=False, async_move=False)
            
            # --- DISK DUMP (Safe to do while hovering) ---
            # Append the status to the data and save it to our master CSV
            for row in current_insertion_data:
                row.append(status) # Add whether this run ultimately passed or failed
                
            # Convert RAM list to a DataFrame and append it to a master CSV file
            df_temp = pd.DataFrame(current_insertion_data, columns=[
                "Test_ID", "Time_s", "TCP_X", "TCP_Y", "TCP_Z", 
                "Force_X", "Force_Y", "Force_Z", "Target_X", "Target_Y", "Status"])
            
            # If it's the first test, create the file and write headers. Otherwise, append.
            if i == 1:
                df_temp.to_csv("monte_carlo_results.csv", index=False, mode='w')
            else:
                df_temp.to_csv("monte_carlo_results.csv", index=False, mode='a', header=False)
            
            time.sleep(1.0)
            
        # --- E. Final Report ---
        print(f"\n========================================")
        print(f"       MONTE CARLO RESULTS")
        print(f"========================================")
        print(f"Total Attempts: {NUM_TESTS}")
        print(f"Successes:      {successes}")
        print(f"Failures (Jams):{failures}")
        
        reliability = (successes / NUM_TESTS) * 100
        print(f"System Reliability: {reliability:.1f}%")
            
    except Exception as e:
        print(f"[ERROR] Execution failed: {e}")
    except KeyboardInterrupt:
        print("\n[WARNING] User aborted test via Keyboard Interrupt.")
        robot.rtde_c.stopJ(2.0)
    finally:
        robot.cleanup()
        print("[INFO] Done.")

if __name__ == "__main__":
    main()
