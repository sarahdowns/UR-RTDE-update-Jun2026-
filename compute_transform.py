import numpy as np
import csv

def load_points_from_csv(csv_file):
    cam_points = []
    robot_points = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            cam_points.append([float(row['Ball_X']), float(row['Ball_Y']), float(row['Ball_Z'])])
            robot_points.append([float(row['TCP_X']), float(row['TCP_Y']), float(row['TCP_Z'])])
    return np.array(cam_points), np.array(robot_points)

def compute_transformation_matrix(cam_points, robot_points):
    # Calculate centroids
    centroid_cam = np.mean(cam_points, axis=0)
    centroid_robot = np.mean(robot_points, axis=0)

    # Center the points
    cam_centered = cam_points - centroid_cam
    robot_centered = robot_points - centroid_robot

    # Compute the rotation using SVD
    H = cam_centered.T @ robot_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Reflection correction
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = centroid_robot - R @ centroid_cam

    # Assemble homogeneous transform
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

# Example usage
csv_file = "zed_ur_tennisball_20250717_152601.csv"  # Replace with your latest CSV file name
cam_pts, robot_pts = load_points_from_csv(csv_file)
T = compute_transformation_matrix(cam_pts, robot_pts)
np.save("cam_to_robot_transform.npy", T)
print("Saved transformation matrix to cam_to_robot_transform.npy")

