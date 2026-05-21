import sys
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2
import time

import csv
import matplotlib.pyplot as plt

rg_id = 0
ip = "192.168.5.5"              # Change to desired robot IP
rg_gripper = RG2(ip,rg_id)

rtde_c = RTDEControl(ip)           
rtde_r = RTDEReceive(ip)

def setup_realtime_plot():
    plt.ion()
    fig, axs = plt.subplots(2, 2, figsize=(5, 6))
    ax1, ax2, ax3, ax4 = axs.flatten()
    line1, = ax1.plot([], [], label="TCP X")
    line2, = ax2.plot([], [], label="Force X", color='green')
    line3, = ax3.plot([], [], label="Force Y", color='red')
    line4, = ax4.plot([], [], label="Force Z", color='orange')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("TCP X (m)")
    ax2.set_ylabel("Force X (N)")
    ax2.set_xlabel("Time (s)")
    ax3.set_ylabel("Force Y (N)")
    ax3.set_xlabel("Time (s)")
    ax4.set_ylabel("Force Z (N)")
    ax4.set_xlabel("Time (s)")
    ax1.legend()
    ax2.legend()
    ax3.legend()
    ax4.legend()

    timestamps = []
    tcp_x = []
    force_x = []
    force_y = []
    force_z = []
    start_time = time.time()

    while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
        current_time = time.time() - start_time
        tcp_pose = rtde_r.getActualTCPPose()
        tcp_force = rtde_r.getActualTCPForce()

        # Append data
        timestamps.append(current_time)
        tcp_x.append(tcp_pose[0])           # Vertical height of end effector. 3rd element in list
        force_x.append(tcp_force[0])
        force_y.append(tcp_force[1])
        force_z.append(tcp_force[2])        # Force exerted up/down on the end effector. 3rd element in list

    # Determine the location of the socket
        if tcp_force[0] < -6.0 and tcp_force[1] < -9 and tcp_force[2] > 0.5:
            print(f"To the Right: Force X ({tcp_force[0]:.4f} N) is < -8 N. Force Y ({tcp_force[3]:.4f} N) is < -9 N. Force Y ({tcp_force[3]:.4f} N) is > 0.5 N. \n Current TCP X-pose: {tcp_pose[0]:.4f} m")

        # if tcp_force[0] > -10.0 and tcp_force[1] > -6.5 and tcp_force[2] > -2.5:
        #     print(f"To the Left: Force X ({tcp_force[0]:.4f} N) is less than -8 N. Force Z ({tcp_force[3]:.4f} N) is greater than 0 N. Current TCP X-pose: {tcp_pose[0]:.4f} m")

        # Update plot
        line1.set_data(timestamps, tcp_x)
        line2.set_data(timestamps, force_x)
        line3.set_data(timestamps, force_y)
        line4.set_data(timestamps, force_z)
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        ax3.relim()
        ax3.autoscale_view()
        ax4.relim()
        ax4.autoscale_view()
        plt.pause(0.05)

        # new_waypoint = rtde_c.getAsyncOperationProgress()
        # if new_waypoint != waypoint:
        #     waypoint = new_waypoint
        #     print(f"Moving to waypoint {waypoint}")

    plt.ioff()
    plt.show()