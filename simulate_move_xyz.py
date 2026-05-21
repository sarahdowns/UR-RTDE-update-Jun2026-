# File name: simulate_pickup_tb.py
# Purpose: Offline simulation of the tennis ball pickup sequence

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Load calibration matrix ---
T_cam_to_robot = np.load("cam_to_robot_transform.npy")

# --- Example detected ball in camera frame ---
# Replace this with your actual detection
ball_cam_xyz = np.array([0.20687141, 0.03275463, 0.63240653, 1.0])  # homogeneous coordinates

# --- Transform to robot frame ---
ball_robot_xyz = (T_cam_to_robot @ ball_cam_xyz)[:3]

print("Camera frame:", ball_cam_xyz[:3])
print("Transformed robot frame:", ball_robot_xyz)

# Visual check
print("Camera Z:", ball_cam_xyz[2])
print("Robot Z (after T_cam_to_robot):", ball_robot_xyz[2])

# --- Robot workspace limits for UR5e (approx) ---
workspace_radius = 1.0  # meters
z_min = 0.0
z_max = 1.2

# --- Plotting ---
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("ZED Ball Detection → UR5e Robot Frame")

# Plot the robot base
ax.scatter(0,0,0, color='black', s=80, label='Robot Base')

# Plot the transformed ball target
ax.scatter(ball_robot_xyz[0], ball_robot_xyz[1], ball_robot_xyz[2],
           color='red', s=120, label='Transformed Ball Target')

# Optional: visualize the raw camera point in its own frame (scaled)
ax.scatter(ball_cam_xyz[0], ball_cam_xyz[1], ball_cam_xyz[2],
           color='orange', s=80, label='Raw Camera Ball (for reference)')

# Draw approximate UR5e workspace sphere
u = np.linspace(0, 2*np.pi, 50)
v = np.linspace(0, np.pi, 25)
x = workspace_radius * np.outer(np.cos(u), np.sin(v))
y = workspace_radius * np.outer(np.sin(u), np.sin(v))
z = z_max * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_wireframe(x, y, z, color='blue', alpha=0.1, linewidth=0.5, label='UR5e Workspace')

# Labels
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_box_aspect([1,1,1])
ax.legend()
ax.view_init(elev=30, azim=45)
plt.show()
