# Author: Sarah Downs
# Program Name: plot_test_grid.py
# This code runs after the calibration program "detect_tennisball_csvCal.py" and the transformation matrix program "detect_tennisball_Calibration.py"
# then after "move_test_grid_ZED.py" to visulaize the workspace and arm's current position

import pandas as pd
import pyzed.sl as sl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from move_xyz_safe import UR5eSafeController

def get_live_ball_pos(zed):
    """Retrieves the live 3D position of the ball from the ZED2i."""
    objects = sl.Objects()
    runtime_params = sl.RuntimeParameters()
    
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve detected objects
        zed.retrieve_objects(objects, sl.ObjectDetectionRuntimeParameters())
        
        if len(objects.object_list) > 0:
            # Get the first detected object (your tennis ball)
            # Position is [x, y, z] in Camera Frame (meters)
            pos = objects.object_list[0].position
            return np.array([pos[0], pos[1], pos[2], 1.0])
    return None
    
def plot_full_scene(csv_file="ur5e_reachability_results.csv", ip="192.168.5.5"):
    # --- 1. Load Grid & Calibration Data ---
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.coordinate_units = sl.UNIT.METER
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Could not open ZED camera.")
        return    
        
    # Enable Object Detection (Assumes you have the model loaded)
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True
    zed.enable_object_detection(obj_param)

    try:
        df = pd.read_csv(csv_file)
        T_cam_to_robot = np.load("cam_to_robot_transform.npy")
        cam_pos = T_cam_to_robot[:3, 3]
        cam_rot = T_cam_to_robot[:3, :3]
    except Exception as e:
        print(f"[ERROR] Missing data files: {e}")
        return

    # --- 2. Define Current Ball Position ---
    ctrl = UR5eSafeController(ip=ip)
    robot = ctrl.robot

    # --- 3. Get Current Ball Position ---
    print("Detecting live ball position...")
    ball_camera_frame = get_live_ball_pos(zed)
    
    if ball_camera_frame is not None:
        # Step A: Transform to World Frame (Relative to floor/gravity)
        ball_world_frame = T_cam_to_robot @ ball_camera_frame
        bx, by, bz = ball_world_frame[:3]
        
        # Step B: Transform to Base Frame (Relative to tilted mounting plate)
        # We take the World position and apply the inverse of the 45-deg Ry tilt
        T_base_to_world = ctrl.robot.base # This is your SE3.Ry(45)
        P_world = SE3(bx, by, bz)
        P_base = T_base_to_world.inv() * P_world
        base_x, base_y, base_z = P_base.t
    else:
        # Fallback to last known if detection fails
        bx, by, bz = -0.184, -0.092, 0.505
        base_x, base_y, base_z = 0.0, 0.0, 0.0 # Placeholder


    # --- 4. Get Live Robot Position ---
    try:
        rtde_r = RTDEReceive(ip)
        current_q = rtde_r.getActualQ()
    except:
        current_q = [0, -np.pi/2, 0, -np.pi/2, 0, 0]

    # --- 5. Create Plot ---
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot Reachability Grid
    reachable = df[df['reachable'] == True]
    unreachable = df[df['reachable'] == False]
    ax.scatter(reachable['x'], reachable['y'], reachable['z'], 
               c='green', marker='o', s=20, alpha=0.3, label='Reachable')
    ax.scatter(unreachable['x'], unreachable['y'], unreachable['z'], 
               c='red', marker='x', s=15, alpha=0.2, label='Unreachable')

    # Plot Robot
    joint_coords = np.array([p.t for p in robot.fkine_all(current_q)])
    ax.plot(joint_coords[:,0], joint_coords[:,1], joint_coords[:,2], 'k-', linewidth=4, label='UR5e Arm')

  # --- 6. Plot Task Prism in WORLD FRAME ---
    # x_b = [-0.8, 1.0]   # Not exact
    # y_b = [-0.4, -1.1] 	# 61 cm
    # z_b = [-0.2, -0.23]	# 3 cm
    # --- 6. Plot Task Prism ---
    xb, yb, zb = [-0.8, 1.0], [-0.4, -1.1], [0.0737, 0.1037]
    for sx in xb:
        for sy in yb: ax.plot([sx, sx], [sy, sy], zb, color='magenta', alpha=0.3)
    for sx in xb:
        for sz in zb: ax.plot([sx, sx], yb, [sz, sz], color='magenta', alpha=0.3)
    for sy in yb:
        for sz in zb: ax.plot(xb, [sy, sy], [sz, sz], color='magenta', alpha=0.3)

    # --- 7. Plot Camera & Tennis Ball ---
    ax.scatter(cam_pos[0], cam_pos[1], cam_pos[2], color='cyan', marker='P', s=150, label='ZED2i')
    
    # Target in World Frame (Relative to Floor - Lawngreen)
    ax.scatter(bx, by, bz, color='lawngreen', marker='o', s=150, edgecolors='black', label='Ball (World Frame)')
    
    # Target in Base Frame (Relative to Arm - Red Ring)
    # Changed label from "Base of Robot" to "Ball (Base Frame)"
    ax.scatter(base_x, base_y, base_z, color='none', marker='o', s=150, 
               edgecolors='red', linewidth=2, label='(Base Frame)')

    # --- Print Summary for your Logs ---
    print("-" * 30)
    print(f"LIVE Ball World Frame: X={bx:.4f}, Y={by:.4f}, Z={bz:.4f}")
    print(f"LIVE Ball Base Frame:  X={base_x:.4f}, Y={base_y:.4f}, Z={base_z:.4f}")
    print("-" * 30) 
    
    # Visual line from Camera to Ball (Line of Sight)
    ax.plot([cam_pos[0], bx], [cam_pos[1], by], [cam_pos[2], bz], 'c--', alpha=0.4, label='Line of Sight')

    # Final Formatting
    print(f"LIVE Ball World XYZ: {base_x:.4f}, {base_y:.4f}, {base_z:.4f}")
    ax.set_title("Full Simulation: Robot, Camera, Task Prism, and Ball Target")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))
    ax.view_init(elev=20, azim=45)
    plt.show()
    

if __name__ == "__main__":
    plot_full_scene()
