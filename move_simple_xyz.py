# Author: Sarah Downs
# Singularity Avoidance

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# --- 1. Configuration ---
ip = "192.168.5.5"
rtde_c = RTDEControl(ip)
rtde_r = RTDEReceive(ip)

# --- 2. Define UR5e Model with Safety Constraints ---
# Values for 'a' and 'd' are sourced from the UR5e technical specifications [cite: 105, 106, 109, 110, 112]
# Joint working ranges are ±360° (±np.pi*2) 
# robot = rtb.DHRobot([
#     rtb.RevoluteDH(d=0.1625, a=0,      alpha=np.pi/2,  qlim=[-np.pi, np.pi]),
#     rtb.RevoluteDH(d=0,      a=-0.425,  alpha=0,       qlim=[-np.pi, 0]),     # Limit shoulder for forward-facing incline
#     rtb.RevoluteDH(d=0,      a=-0.3922, alpha=0,       qlim=[-np.pi, np.pi]), 
#     rtb.RevoluteDH(d=0.1333, a=0,      alpha=np.pi/2,  qlim=[-np.pi, np.pi]),
#     rtb.RevoluteDH(d=0.0997, a=0,      alpha=-np.pi/2, qlim=[-np.pi, np.pi]),
#     rtb.RevoluteDH(d=0.0996, a=0,      alpha=0,        qlim=[-np.pi, np.pi])
# ], name="UR5e_Safe")

# Update the last link (d6) to include the gripper length
tool_offset= 0
total_d6 = 0.0996 + tool_offset 
robot = rtb.DHRobot([
        rtb.RevoluteDH(d=0.1625, a=0,      alpha=np.pi/2,  qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=0,      a=-0.425,  alpha=0,       qlim=[-np.pi, 0]),
        rtb.RevoluteDH(d=0,      a=-0.3922, alpha=0,       qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=0.1333, a=0,      alpha=np.pi/2,  qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=0.0997, a=0,      alpha=-np.pi/2, qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=total_d6, a=0,    alpha=0,        qlim=[-np.pi, np.pi]) # Modified d6
    ], name="UR5e_Safe_with_Tool")

# Apply 45-degree base tilt relative to gravity 
robot.base = SE3.Ry(np.deg2rad(45))

def move_to_xyz_safe(x, y, z, visualize=True):
    home_q = rtde_r.getActualQ()		# Capture starting configuration as "home"
    
   T_target = SE3(x, y, z) * SE3.RPY(0, np.pi, 0)
    
    # 2. Define a Diagonal Gain Matrix
    # We give high weight (1.0) to X, Y, Z and lower weight (0.1) to orientation.
    # This allows the solver to "sacrifice" orientation accuracy to satisfy the position.
    W = np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
    
    # 3. Solve IK using the Weighting matrix
    # Note: Using 'mask' with ikine_LM is binary, but we can use q0 to bias it.
    # For true partial constraints, we use the numerical solver with a weight matrix.
    sol = robot.ikine_LM(T_target, q0=home_q, mask=[1,1,1,0.2,0.2,0.2])

    if sol.success:
        # --- 1. Deviation Analysis ---
        # Calculate the actual pose reached by the IK solver
        T_reached = robot.fkine(sol.q)
        
        # Calculate the angular error (difference between 'ideal' down and 'actual')
        # We compare the Z-axis of the tool to the world Z-axis
        tool_z_axis = T_reached.R[:, 2]  # The third column of the rotation matrix
        ideal_z_axis = np.array([0, 0, -1]) # Straight down in world frame
        
        # Dot product to find the angle (cosine similarity)
        cos_theta = np.dot(tool_z_axis, ideal_z_axis)
        angle_err = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

        # --- 2. Safety & Singularity Checks ---
        # Check for potential self-collision (Joint 3 proximity)
        if abs(sol.q[2]) < np.deg2rad(15):
            print(f"[DANGER] Potential self-collision. Joint 3 at {np.degrees(sol.q[2]):.2f}°. Aborting.")
            return False

        # Check for Wrist Singularity (Joint 5 near 0)
        if abs(sol.q[4]) < np.deg2rad(5):
            print("[WARNING] Close to Wrist Singularity. The flexible orientation helped, but proceed with caution.")

        # --- 3. User Confirmation & Execution ---
        if visualize:
            print(f"[VISUAL] Orientation Tilt: {angle_err:.2f}° from vertical.")
            print("[INFO] Close the plot window to proceed to physical movement.")
            robot.plot(sol.q, block=True)
        
        prompt = f"Move to ({x}, {y}, {z}) with {angle_err:.1f}° tilt? (y/n): "
        if input(prompt).lower() == 'y':
            # Execute move using moderate operating settings
            # Velocity: 0.3 rad/s, Acceleration: 1.2 rad/s^2
            rtde_c.moveJ(sol.q, 0.3, 1.2)
            print("[STATUS] Movement complete.")
            
            if input("Return to home position? (y/n): ").lower() == 'y':
                print("Returning home...")
                rtde_c.moveJ(home_q, 0.3, 1.2)
                print("[STATUS] Back at initial position.")
            return True
        else:
            print("[ABORT] Movement cancelled by user.")
            return False
    else:
        print(f"[ERROR] Could not find a valid IK solution for Point({x}, {y}, {z}).")
        print("Try increasing the allowable tilt or checking the robot's reach envelope.")
        return False

if __name__ == "__main__":
    try:
        # Example: Moving to a point in the workspace
        # The solver will now tilt the gripper if it helps avoid a singularity
        move_to_xyz_flexible(0.3, -0.4, 0.2)
    finally:
        # Always ensure the RTDE script is stopped to release control of the UR5e
        rtde_c.stopScript()
        print("[INFO] RTDE interface closed.")
