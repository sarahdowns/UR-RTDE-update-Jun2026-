# Author
# File name: move_tennisball_median.py

import os
import glob
import numpy as np
import pandas as pd
import datetime
import time
from spatialmath import SE3
from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2

# System Configuration
ip = "192.168.5.5"
rg_id = 0
CALIB_FILE = "cam_to_robot_transform.npy"
HOVER_OFFSET = 0.1  # 10 cm above the ball for safety
GRIPPER_FORCE = 25.0

rg_gripper = RG2(ip, rg_id)

pitch_deg = 0
roll_deg = -90
yaw_deg= 90

def load_latest_csv(folder='.'):
    """Finds the most recently generated camera data file."""
    csv_files = glob.glob(os.path.join(folder, "tb_camxyz_*.csv"))
    if not csv_files:
        raise FileNotFoundError("No camera data CSV found. Run your capture script first!")
    return max(csv_files, key=os.path.getctime)

def main():
    # 1. Load Data and Calculate Median
    data_file = load_latest_csv()
    print(f"[INFO] Loading vision data from: {data_file}")
    
    # Read CSV and calculate the median for each axis
    df = pd.read_csv(data_file)
    p_cam = np.array([
        df['Ball_X'].median(), 
        df['Ball_Y'].median(), 
        df['Ball_Z'].median()
    ])
    print(f"[INFO] Camera Frame Median: X={p_cam[0]:.4f}, Y={p_cam[1]:.4f}, Z={p_cam[2]:.4f}")

    # 2. Transform to the Robot's Tilted World Frame
    if not os.path.exists(CALIB_FILE):
        print(f"[ERROR] Calibration matrix {CALIB_FILE} not found.")
        return
        
    T_cam_to_robot = np.load(CALIB_FILE)
    T_world_base = SE3.Ry(np.deg2rad(45))

    # Convert to homogeneous coordinate and multiply
    p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
    p_base = T_cam_to_robot @ p_cam_h
    p_world = T_world_base.A @ np.array([p_base[0], p_base[1], p_base[2], 1.0])
    
    target_x, target_y, target_z = p_world[:3]
    print(f"[INFO] Robot World Target:  X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")

    # 3. Execute Physical Movement
    print("\n[INIT] Connecting to UR5e...")
    robot = UR5eSafeController(ip)

    try:
        # Step A: Move to Hover and Open Gripper
        print(f"\n[ACTION] Moving to safe hover position (+{HOVER_OFFSET*100} cm)...")
        rg_gripper.rg_grip(100.0, GRIPPER_FORCE) 
        
        robot.move_to_xyz_safe(target_x, target_y, target_z + HOVER_OFFSET, 
                               visualize=False, roll_deg = roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg)
        
        # Step B: Final Approach to the Equator
        user_input = input("\n[PROMPT] Hover complete. Dive to ball equator and grip? (y/n): ")
        if user_input.lower() == 'y':
            
            # The exact height of the tennis ball's center
            ball_equator_z = 0.0335  
            
            print(f"[ACTION] Lowering to ball equator...")
            robot.move_to_xyz_safe(target_x, target_y, target_z, 
                                   visualize=False, roll_deg = roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg)
            
            # Step C: Grasp and Lift
            print(f"[ACTION] Grasping ball...")
            rg_gripper.rg_grip(55.0, GRIPPER_FORCE) # Target 55mm to ensure a tight squeeze
            time.sleep(1.0) # Let the jaws close
            
            print(f"[ACTION] Lifting ball...")
            robot.move_to_xyz_safe(target_x, target_y, target_z + HOVER_OFFSET, 
                                   visualize=False, roll_deg = roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg)
            
            print("\n[SUCCESS] Tennis ball pick up achieved!")
            
    except Exception as e:
        print(f"[ERROR] Execution failed: {e}")
    finally:
        robot.cleanup()
        print("[INFO] Done.")

if __name__ == "__main__":
    main()
