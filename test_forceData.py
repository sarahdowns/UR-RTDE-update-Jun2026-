# Author: Sarah Downs
# This code measures and plots the realtime force data placed on the end effector. We
# used this to test the accuracy of the force measures and pressure from another bot

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2
import time

import csv
import matplotlib.pyplot as plt

rg_id = 0
ip = "192.168.5.5"             # Change to desired robot IP
rg_gripper = RG2(ip,rg_id)

rtde_c = RTDEControl(ip)
rtde_r = RTDEReceive(ip)

rg_width = rg_gripper.get_rg_width()

init_q = rtde_r.getActualQ()

# path = Path() # Not needed if no movements
vel = 0.3
acc = 1.8
blend = 0.099

# Starting Gripper width
# target_force = 30.00
# rg_gripper.rg_grip(50, target_force)
# print("Starting width: ",rg_width)
# time.sleep(.5)

print("Starting the plot...")

print("Plotting data for 10 seconds... ")

plt.ion()
fig, axs = plt.subplots(2, 2, figsize=(5, 6))
ax1, ax2, ax3, ax4 = axs.flatten()
line1, = ax1.plot([], [], label="TCP Z")
line2, = ax2.plot([], [], label="Force X", color='green')
line3, = ax3.plot([], [], label="Force Y", color='red')
line4, = ax4.plot([], [], label="Force Z", color='orange')
ax1.set_ylabel("TCP Z (m)")
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
tcp_z = []
force_x = []
force_y = []
force_z = []
start_time = time.time()

duration = 2000 # seconds

# Loop for a fixed duration of 10 seconds
while (time.time() - start_time) < duration:
    current_time = time.time() - start_time
    tcp_pose = rtde_r.getActualTCPPose()
    tcp_force = rtde_r.getActualTCPForce()

    # --- FIX Y-AXIS LIMITS HERE ---
    # Replace these values with ranges appropriate for your setup
    ax1.set_ylim(0.0, 50.0)  # Example: TCP Z from 0.0 to 1.0 meters
    ax2.set_ylim(-120.0, 50.0) # Example: Force X from -10 to 10 Newtons
    ax3.set_ylim(-50.0, 50.0) # Example: Force Y from -10 to 10 Newtons
    ax4.set_ylim(-60.0, 50.0) # Example: Force Z from -20 to 20 Newtons (often higher for gravity)
    # --- END Y-AXIS LIMITS FIX ---

    # Append data
    timestamps.append(current_time)
    tcp_z.append(tcp_pose[2])           # Vertical height of end effector. 3rd element in list
    force_x.append(tcp_force[0])
    force_y.append(tcp_force[1])
    force_z.append(tcp_force[2])        # Force exerted up/down on the end effector. 3rd element in list


    # Update plot
    line1.set_data(timestamps, tcp_z)
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

plt.ioff()
plt.show()

print("Plotting finished after 10 seconds.\n")

rtde_c.stopScript()
print("Script stopped.")


#### Note ####
# TCP Pose
# Out: [ , ,  ,  ,   , +]

# Joint Pose
# CW: [ , ,  ,  , - , +]