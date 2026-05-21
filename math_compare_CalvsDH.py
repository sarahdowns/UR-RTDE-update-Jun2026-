# Program name: math_compare_CalvsDH.py
# compare the Robot's actual position to your model's prediction in the same frame by calculating 
# the FK. If the X and Z mismatch but Y is the same, then you're DH 
# parameters and/or offset value are likely incorrect 

from move_xyz_safe import UR5eSafeController
from spatialmath import SE3
import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface

ip = "192.168.5.5"
rtde_r = RTDEReceiveInterface(ip)

# Load your controller
ctrl = UR5eSafeController(ip=ip)

actual_tcp = rtde_r.getActualTCPPose()[:3]
actual_joints = rtde_r.getActualQ()

# 2. Calculate Model FK WITHOUT the base tilt
# Temporarily set the base to an Identity matrix (no rotation)
original_base = ctrl.robot.base
ctrl.robot.base = SE3() 
model_fk = ctrl.robot.fkine(actual_joints)
ctrl.robot.base = original_base # Restore tilt

print("--- Kinematic Comparison (Base Frame) ---")
print(f"Robot Actual TCP: {actual_tcp}")
print(f"Model FK Result:  {model_fk.t.round(4)}")
print("-----------------------------------------")

error = np.linalg.norm(np.array(actual_tcp) - model_fk.t)
print(f"Total Euclidean Error: {error*1000:.2f} mm")
print("="*50 + "\n")

# 5. Move to a NEW coordinate
confirm = input("Would you like to test a move to a NEW coordinate? (y/n): ")
if confirm.lower() == 'y':
    try:
        # Prompt for target coordinates (Base Frame)
        tx = 0.0214
        ty = -0.6183
        tz = 0.1860
        
        # 1. Define target pose (keeping gripper pointing straight down)
        # We use SE3.RPY(0, np.pi, 0) to keep the gripper vertical
        T_target = SE3(tx, ty, tz) * SE3.RPY(0, np.pi, 0)
        
        # 2. Solve Inverse Kinematics
        # We use your current joints as a 'seed' so the robot picks the closest solution
        sol = ctrl.robot.ikine_LM(T_target, q0=actual_joints)
        
        if sol.success:
            print(f"--- IK Success ---")
            print(f"Target Joints (deg): {np.rad2deg(sol.q).round(1)}")
            
            # 3. Command the move
            speed = 0.2  # Keep it slow for the first test
            accel = 0.1
            ctrl.rtde_c.moveJ(sol.q, speed, accel)
            print("Move complete.")
        else:
            print("[ERROR] Could not find a valid kinematic solution for that point.")
            
    except ValueError:
        print("Invalid input. Please enter numbers for X, Y, and Z.")

ctrl.cleanup()

ctrl.cleanup() 
