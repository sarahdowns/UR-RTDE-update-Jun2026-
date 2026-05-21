# Author: Sarah Downs
# Move robot arms to a position above the table and receive coordinates from the 
# zed camera, pick up object. 

##### Tried to reduce the number of lines by creating plot_realtime separatly but doesn't
##### seem to work

import sys
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2
from plot_TCP_path import visualize_tcp_path
from plot_realtime import setup_realtime_plot
import time

import csv
import matplotlib.pyplot as plt

rg_id = 0
ip = "192.168.5.5"              # Change to desired robot IP
rg_gripper = RG2(ip,rg_id)

rtde_c = RTDEControl(ip)           
rtde_r = RTDEReceive(ip)

rg_width = rg_gripper.get_rg_width()
init_q = rtde_r.getActualQ()

path = Path()
vel = 0.5
acc = 2.0
blend = 0.099

## Open a CSV file
log_file = open("robot_log.csv", mode="w", newline="")
log_writer = csv.writer(log_file)
log_writer.writerow(["timestamp", "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz", "force_x", "force_y", "force_z"])

def adjust_rod (rtde_r, rtde_c):
    print("Running rod adjustment...")
    time.sleep(2.0)

# Starting Gripper width
target_force = 30.00
rg_gripper.rg_grip(50, target_force)
print("Starting width: ",rg_width)
time.sleep(.5)

## Poole Positions
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.156, 0.1579, 0.4248, -1.295, -0.198, -0.895, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.202, 0.159, 0.655, -1.317, -0.167, -0.895, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.0, -1.5, -2.0, -1.643, 2.679, 0.014, 1.0, acc, 0.0]))  
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.18, -0.7977, 0.5287, 0.2652, 2.2483, -0.9034, vel, acc, 0.0]))
# path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.2029, -0.7868, 0.5261, 0.2335, 2.2207, -0.8438, vel, acc, 0.0]))  

# ### Plot rod
# ### STEP 1: Move to socket position ###
pre_grip_path = path  # Use your existing `path` object
pre_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.2674, -0.7977, 0.4412, 0.2652, 2.2483, -0.9035, vel, acc, 0.0]))        # Lower end effector to grasp
# pre_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [ 0.2832, -0.7868, 0.4458, 0.2336, 2.2207, -0.8438, vel, acc, 0.0]))        # Lower end effector to grasp

print("Moving to socket position...")
rtde_c.movePath(pre_grip_path, True)
# visualize_tcp_path(rtde_r, rtde_c)

# Wait until robot reaches position
while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.1)

    tcp_pose = rtde_r.getActualTCPPose()
    tcp_force = rtde_r.getActualTCPForce()
    timestamp = time.time()

    log_writer.writerow([
        f"{timestamp:.4f}",
        f"{tcp_pose[0]:.4f}", f"{tcp_pose[1]:.4f}", f"{tcp_pose[2]:.4f}",
        f"{tcp_pose[3]:.4f}", f"{tcp_pose[4]:.4f}", f"{tcp_pose[5]:.4f}",
        f"{tcp_force[0]:.4f}", f"{tcp_force[1]:.4f}", f"{tcp_force[2]:.4f}"
    ])

## STEP 2: Perform the gripping action ###
print("Reached socket position. Inserting rod...")
rg_gripper.rg_grip(20, target_force)
time.sleep(1)

# STEP 3: Continue with rest of the movement ###
post_grip_path = Path()
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.1361, -0.7977, 0.5586, 0.2652, 2.2483, -0.9034, vel, acc, 0.0]))
# post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.1656, -0.7868, 0.5633, 0.2339, 2.2205, -0.8445, vel, acc, 0.0]))

post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.1024, -0.7977, 0.5249, 0.2652, 2.2483, -0.9034, vel, acc, 0.0]))       # move to the stage-left
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.6793, -2.1625, -1.4608, 0.4741, 2.4833, 1.5092, vel, acc, 0.0]))
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.6092, -2.1862, -1.5410, 0.5785, 2.4128, 1.5100, vel, acc, 0.0]))
adjust_rod(rtde_r, rtde_c)

print("Searching for socket...")
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.6055, -2.1834, -1.5419, 0.5767, 2.4092, 1.5100, vel, acc, 0.0]))
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.4895, -2.0854, -1.5327, 0.4699, 2.2933, 1.5110, vel, acc, 0.0]))
# move left
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.5467, -2.1345, -1.5464, 0.5325, 2.3504, 1.5105, 0.1, 1.0, 0.0]))
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.5269, -2.1166, -1.5430, 0.5113, 2.3306, 1.5107, 0.1, 1.0, 0.0]))
# # Straighten rod
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.5832, -2.0968, -1.4835, 0.4317, 2.3872, 1.5101, 0.1, 1.0, 0.0]))
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.3379, -1.8301, -1.2138, -0.6499, 1.9323, 0.7308, 0.1, 1.0, 0.0]))
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.1852, -1.8541, -1.4136, -0.4022, 1.8009, 0.8135, 0.1, 1.0, 0.0]))

print("Post grip movement...")
rtde_c.movePath(post_grip_path, True)

######## PATH MOVEMENT ################################
# Wait for start of asynchronous operation
while not rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.010)
print("Async path started.. ")

setup_realtime_plot()

rg_gripper.rg_grip(90, target_force)   
print("Rod released.")
time.sleep(1)                           # Add a small delay to ensure the gripper opens
return_path = Path()
return_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.0354, -1.4877, -1.8264, -0.3177, 1.7051, 0.8535, 0.2, acc, 0.0]))

print("Starting return movements...")
rtde_c.movePath(return_path, True) # 'True' means synchronous execution

while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.05)

rtde_c.moveJ(init_q)                     # Return to intial position
print("Returning to initial position...")                         
log_file.close()     
print("CSV file Closed.")              
rtde_c.stopScript()


#### Note ####
# TCP Pose
# Out: [ , ,  ,  ,   , +]

# Joint Pose
# CW: [ , ,  ,  , - , +]


def adjust_rod (rtde_r, rtde_c):
    print("Running rod adjustment...")