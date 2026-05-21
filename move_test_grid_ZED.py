# Author: Sarah Downs
# Program Name: move_test_grid_ZED.py
# This code runs after the calibration program "detect_tennisball_csvCal.py" and the transformation matrix program "detect_tennisball_Calibration.py"
# to determine the workspace of the arm based a 3D grid of points, the camera, and the arm's current position. Change the range and point values as needed. 
	# Total Points Tested:
	# Reachability Score:  %
	# Average Tilt Needed:  %

import numpy as np
import pandas as pd
from spatialmath import SE3
from move_xyz_safe import UR5eSafeController  # Importing your central model

def run_grid_analysis():
    # --- 1. Initialize Controller ---
    # This automatically loads the 45-deg tilt and diagram measurements
    ctrl = UR5eSafeController(ip="192.168.5.5")
    robot = ctrl.robot

    # --- 2. Define the Test Grid ---
    # Adjusted to provide a dense 1000-point cloud for your workspace mapping
    x_range = np.linspace(-0.5, 1.0, 10)
    y_range = np.linspace(-1.1, 0.2, 10)
    z_range = np.linspace(-0.2, 1.0, 10)

    results = []
    print(f"Starting reachability test for {len(x_range)*len(y_range)*len(z_range)} points...")

    # --- 3. Iterate Through the Grid ---
    for x in x_range:
        for y in y_range:
            for z in z_range:
                # Target Pose: Gripper pointing straight down relative to floor
                T_target = SE3(x, y, z) * SE3.RPY(0, np.pi, 0)
                
                # Weight mask: [X, Y, Z, Roll, Pitch, Yaw] 
                # Matches the logic in move_xyz_safe.py
                W = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
                
                # Use the class-defined robot model for the IK solver
                sol = robot.ikine_LM(T_target, mask=W)
                
                reachable = sol.success
                tilt = 0.0
                joint_3_collision = False
                
                if reachable:
                    # Calculate Tilt Angle relative to pure vertical
                    T_reached = robot.fkine(sol.q)
                    tool_z = T_reached.R[:, 2]
                    ideal_z = np.array([0, 0, -1])
                    tilt = np.degrees(np.arccos(np.clip(np.dot(tool_z, ideal_z), -1.0, 1.0)))
                    
                    # Safety check: Prevent the "Elbow" from folding into itself
                    if abs(sol.q[2]) < np.deg2rad(10): # Using your move_xyz_safe limit
                        joint_3_collision = True
                
                results.append({
                    'x': x, 'y': y, 'z': z,
                    'reachable': reachable and not joint_3_collision,
      move              'tilt_deg': tilt,
                    'j3_danger': joint_3_collision
                })

    # --- 4. Export & Summary ---
    df = pd.DataFrame(results)
    df.to_csv("ur5e_reachability_results.csv", index=False)

    reach_pct = (df['reachable'].sum() / len(df)) * 100
    avg_tilt = df[df['reachable'] == True]['tilt_deg'].mean()

    print("\n" + "-" * 30)
    print("WORKSPACE ANALYSIS COMPLETE")
    print("-" * 30)
    print(f"Total Points Tested: {len(df)}")
    print(f"Reachability Score:  {reach_pct:.1f}%")
    print(f"Average Tilt Needed: {avg_tilt:.2f} degrees")
    print("Results saved to 'ur5e_reachability_results.csv'")
    
    ctrl.cleanup() # Close RTDE interfaces

if __name__ == "__main__":
    run_grid_analysis()
