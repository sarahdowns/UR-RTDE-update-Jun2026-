# Author: Sarah Downs
# THIRD STEP
# Moves robot arm to ball

# Progress Jan 13, 2025: The arm will move smoothly from home to above the ball or close to it. Offset in X required and inconsistent. 

import cv2
import numpy as np
import pyzed.sl as sl
import time

# Import detection function and custom controller
from tennisball_detection import detect_ball_position_from_zed
from gripper_RG2 import RG2
from move_xyz_safe import UR5eSafeController

# --- 1. Configuration & Initialization ---
ip = "192.168.5.5"
rg_id = 0
rg_gripper = RG2(ip, rg_id)

# Initialize Safe Controller (Handles 45-degree base tilt and UR5e kinematics)
arm = UR5eSafeController(ip=ip)

# --- 2. Load Calibration Data ---
try:
    T_cam_to_robot = np.load("cam_to_robot_transform.npy")
    print("Loaded transformation matrix.")
except FileNotFoundError:
    print("ERROR: cam_to_robot_transform.npy not found!")
    exit()

# --- 3. Detection & Verification Loop ---
ball_pos_cam = None

while True:
    print("Scanning for tennis ball... (Close the 'Verification' window to retry)")
    # This call handles the ZED camera lifecycle and returns [X, Y, Z]
    point = detect_ball_position_from_zed(display=True)

    if point is not None:
        # Step 3a: Verification Prompt
        # Note: 'display=True' in detect_ball_position_from_zed will already show the frame.
        # We add a console prompt here to "Pause" and confirm.
        print(f"\n[DETECTED] Ball found at Camera XYZ: {point}")
        user_choice = input("Is this detection correct? (y = Proceed / n = Retake Image): ").lower()
        
        if user_choice == 'y':
            # Convert to homogeneous coordinates for transformation
            ball_pos_cam = np.array([point[0], point[1], point[2], 1.0])
            break # Exit loop and proceed to movement
        else:
            print("[RETRY] Discarding current detection. Restarting scan...")
            continue # Loop back to take another image
    else:
        print("[TIMEOUT] No ball detected. Retrying...")

# --- 4. Coordinate Transformation ---
ball_pos_robot = (T_cam_to_robot @ ball_pos_cam)[:3]

X_BIAS = 0 
Y_BIAS = 0
Z_BIAS = 0

corrected_x = ball_pos_robot[0] + X_BIAS 
corrected_y = ball_pos_robot[1] + Y_BIAS 
corrected_z = ball_pos_robot[2] + Z_BIAS

print(f"[DIAG] Corrected World XYZ: {corrected_x:.4f}, {corrected_y:.4f}, {corrected_z:.4f}")

print(f"[DIAG] Corrected World XYZ: {corrected_x:.4f}, {corrected_y:.4f}, {corrected_z:.4f}")

# --- 5. Movement Execution ---
# Capture the starting position (Initial State) before we begin
initial_home_q = arm.rtde_r.getActualQ()
rg_gripper.rg_grip(60, 25.0)

SAFE_HOVER_Z = max(corrected_z + 0, 0.25) 
PICK_Z = corrected_z + 0

try:
    # STEP 1: Move to Hover
    print("Moving to Hover position...")
    # arm.move_to_xyz_safe(ball_pos_robot[0], ball_pos_robot[1], SAFE_HOVER_Z, visualize=False)
    arm.move_to_xyz_safe(corrected_x, corrected_y, SAFE_HOVER_Z, visualize=False)

    # STEP 2: Lower to Pick
    print("Lowering to Pick position...")
    # arm.move_to_xyz_safe(ball_pos_robot[0], ball_pos_robot[1], PICK_Z, visualize=False)
    arm.move_to_xyz_safe(corrected_x, corrected_y, PICK_Z, visualize=False)

    # STEP 3: Close Gripper
    print("Picking ball...")
    rg_gripper.rg_grip(30, 25.0)
    time.sleep(0.5)

    # STEP 4: Lift back to Hover
    print("Lifting ball...")
    # arm.move_to_xyz_safe(ball_pos_robot[0], ball_pos_robot[1], SAFE_HOVER_Z, visualize=False)
    arm.move_to_xyz_safe(corrected_x, corrected_y, SAFE_HOVER_Z, visualize=False)
    
    # STEP 5: RETURN TO INITIAL POSITION
    print("\n" + "="*30)
    user_home = input("Task complete. Return to initial starting position? (y/n): ").lower()
    if user_home == 'y':
        print("Returning to initial position...")
        # Using a slightly slower speed for the return trip while carrying the ball
        arm.rtde_c.moveJ(initial_home_q, 0.2, 1.0)
        print("[STATUS] Returned to start.")
    else:
        print("[INFO] Staying at current hover position.")

finally:
    # Safely close RTDE connections
    arm.cleanup()
