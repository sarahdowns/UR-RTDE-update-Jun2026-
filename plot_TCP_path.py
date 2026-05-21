# visualize_tcp_path.py

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

def visualize_tcp_path(rtde_r, rtde_c):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D TCP Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Set fixed axis limits based on your workspace (adjust as needed!)
    ax.set_xlim(0.0, 0.8)
    ax.set_ylim(-1.0, 0.5)
    ax.set_zlim(0.3, 0.7)

    tcp_x = []
    tcp_y = []
    tcp_z = []

    print("Launching 3D TCP visualization...")

    while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
        tcp_pose = rtde_r.getActualTCPPose()
        tcp_x.append(tcp_pose[0])
        tcp_y.append(tcp_pose[1])
        tcp_z.append(tcp_pose[2])

        ax.clear()
        ax.set_title("3D TCP Path")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")

        # Fixed axis limits again after clearing
        ax.set_xlim(0.0, 0.8)
        ax.set_ylim(-1.0, 0.5)
        ax.set_zlim(0.3, 0.7)

        ax.plot(tcp_x, tcp_y, tcp_z, label="TCP Path", color='blue')
        ax.legend()
        plt.pause(0.05)

    plt.ioff()
    plt.show()
