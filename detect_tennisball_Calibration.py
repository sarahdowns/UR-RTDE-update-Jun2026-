# Author: Sarah Downs
# Program name: detect_tennisball_Calibration.py
# Second
# This code uses the csv file output from detect_tennisball_csvCal.py to calibrate the system for the
# transformation matrix. This is to identify the matrix between the camera and robot frames. 

import numpy as np
import csv
import os
from glob import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

print("Running calibration...")

def load_latest_csv(folder='.'):
    csv_files = glob(os.path.join(folder, "zed_ur_tennisball_*.csv"))
    if not csv_files:
        raise FileNotFoundError("No matching CSV files found.")
    latest_file = max(csv_files, key=os.path.getctime)
    print(f"[INFO] Using file: {latest_file}")
    return latest_file

def read_points_from_csv(filename):
    ball_points = []
    tcp_points = []
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            ball = [float(row["Ball_X"]), float(row["Ball_Y"]), float(row["Ball_Z"])]
            tcp = [float(row["TCP_X"]), float(row["TCP_Y"]), float(row["TCP_Z"])]
            ball_points.append(ball)
            tcp_points.append(tcp)
    return np.array(ball_points), np.array(tcp_points)
    
def cam_to_robot(p, T):
    p_h = np.array([p[0], p[1], p[2], 1.0])
    p_r = T @ p_h
    return p_r[:3]

def compute_transformation(camera_points, robot_points):
    assert camera_points.shape == robot_points.shape
    N = camera_points.shape[0]

    centroid_cam = np.mean(camera_points, axis=0)
    centroid_tcp = np.mean(robot_points, axis=0)

    cam_centered = camera_points - centroid_cam
    tcp_centered = robot_points - centroid_tcp

    H = cam_centered.T @ tcp_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Reflection case
    if np.linalg.det(R) < 0:
        print("[WARNING] Reflection detected. Fixing.")
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_tcp - R @ centroid_cam

    # Assemble homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T
    

if __name__ == "__main__":

    latest_csv = load_latest_csv(".")
    camera_pts, robot_pts = read_points_from_csv(latest_csv)

    if camera_pts.shape[0] < 3:
        print("[ERROR] Need at least 3 points")
        exit(1)

    T = compute_transformation(camera_pts, robot_pts)
    cam_pts_h = np.hstack((camera_pts, np.ones((camera_pts.shape[0], 1))))
    transformed_pts = (T @ cam_pts_h.T).T[:, :3]
    errors = np.linalg.norm(robot_pts - transformed_pts, axis=1)
    rmse = np.sqrt(np.mean(errors**2))

    print("\n=== Camera → Robot Transform ===")
    print(T)
    print(f"\n[QUALITY ASSURANCE]")
    print(f"Mean Alignment Error: {np.mean(errors)*1000:.2f} mm")
    print(f"Max Alignment Error:  {np.max(errors)*1000:.2f} mm")
    print(f"RMS Alignment Error:  {rmse*1000:.2f} mm")

    # Save
    np.save("cam_to_robot_transform.npy", T)
    print("[INFO] Saved transform to file")
    
### Visulaization ###
    R = T[:3, :3]
    t = T[:3, 3]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Calibration Result: Camera vs Robot Frames")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Plot robot base frame (solid RGB)
    origin = np.array([[0, 0, 0]])
    axes = np.eye(3) * 0.1
    ax.quiver(origin[:,0], origin[:,1], origin[:,2],
              axes[0,0], axes[1,0], axes[2,0], color='r', label='Robot X')
    ax.quiver(origin[:,0], origin[:,1], origin[:,2],
              axes[0,1], axes[1,1], axes[2,1], color='g', label='Robot Y')
    ax.quiver(origin[:,0], origin[:,1], origin[:,2],
              axes[0,2], axes[1,2], axes[2,2], color='b', label='Robot Z')
              
    origin_cam = np.array([0, 0, 0])
    ax.quiver(origin_cam[0], origin_cam[1], origin_cam[2],
              0.05, 0, 0, color='r', linestyle='dashed', label='Cam X')
    ax.quiver(origin_cam[0], origin_cam[1], origin_cam[2],
              0, 0.05, 0, color='g', linestyle='dashed', label='Cam Y')
    ax.quiver(origin_cam[0], origin_cam[1], origin_cam[2],
              0, 0, 0.05, color='b', linestyle='dashed', label='Cam Z')


    # Camera frame (dashed RGB)
    cam_origin = t.reshape(1, 3)
    cam_axes = R * 0.1
    ax.quiver(cam_origin[:,0], cam_origin[:,1], cam_origin[:,2],
              cam_axes[0,0], cam_axes[1,0], cam_axes[2,0], color='r', linestyle='dashed')
    ax.quiver(cam_origin[:,0], cam_origin[:,1], cam_origin[:,2],
              cam_axes[0,1], cam_axes[1,1], cam_axes[2,1], color='g', linestyle='dashed')
    ax.quiver(cam_origin[:,0], cam_origin[:,1], cam_origin[:,2],
              cam_axes[0,2], cam_axes[1,2], cam_axes[2,2], color='b', linestyle='dashed')

    # Transform camera points → robot frame
    cam_pts_h = np.hstack((camera_pts, np.ones((camera_pts.shape[0], 1))))
    transformed_pts = (T @ cam_pts_h.T).T[:, :3]

    # Plot calibration points
    ax.scatter(robot_pts[:,0], robot_pts[:,1], robot_pts[:,2],
               color='blue', label='TCP (Robot frame)', s=40)
    ax.scatter(camera_pts[:,0], camera_pts[:,1], camera_pts[:,2],
               color='orange', label='Ball (Camera frame, before transform)', s=40)
    ax.scatter(transformed_pts[:,0], transformed_pts[:,1], transformed_pts[:,2],
               color='green', marker='^', label='Ball (transformed to robot frame)', s=60)

    # Label each pair of corresponding points
    for i in range(len(robot_pts)):
        ax.text(robot_pts[i,0], robot_pts[i,1], robot_pts[i,2],
                f"R{i+1}", color='blue', fontsize=9)
        ax.text(camera_pts[i,0], camera_pts[i,1], camera_pts[i,2],
                f"C{i+1}", color='orange', fontsize=9)
        ax.text(transformed_pts[i,0], transformed_pts[i,1], transformed_pts[i,2],
                f"T{i+1}", color='green', fontsize=9)
        # Optional: connect transformed camera points to robot points for visual pairing
        ax.plot([transformed_pts[i,0], robot_pts[i,0]],
                [transformed_pts[i,1], robot_pts[i,1]],
                [transformed_pts[i,2], robot_pts[i,2]],
                color='gray', linestyle='--', linewidth=0.8)

    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1.0))
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=25, azim=45)
    plt.show()
