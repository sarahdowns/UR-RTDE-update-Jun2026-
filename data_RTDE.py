import numpy as np
import cv2
import pyzed.sl as sl
import csv
import os
import datetime
import time
import roboticstoolbox as rtb
from spatialmath import SE3
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2

from rtde_receive import RTDEReceiveInterface

# Initialize RTDE
ip = "192.168.5.5"
rtde_r = RTDEReceiveInterface(ip)  # Your robot IP
ipCam = "192.168.5.4"
rtde_r_cam = RTDEReceiveInterface(ipCam)  # Your camera robot IP

tcp = rtde_r.getActualTCPPose()
joints = rtde_r.getActualQ()
tcp_cam = rtde_r_cam.getActualTCPPose()
joints_cam = rtde_r_cam.getActualQ()

# Format to 4 decimals and join with a comma
tcp_string = ", ".join([f"{x:.4f}" for x in tcp])
joints_string = ", ".join([f"{q:.4f}" for q in joints])
tcp_string_cam = ", ".join([f"{x:.4f}" for x in tcp_cam])
joints_string_cam = ", ".join([f"{q:.4f}" for q in joints_cam])

print(f"[DATA] TCP Pose (XYZRPY): [{tcp_string}]")
print(f"[DATA] Joint Angles (rad): [{joints_string}]")
print("-"* 30)
print(f"[DATA] TCP Pose (XYZRPY) camera bot: [{tcp_string_cam}]")
print(f"[DATA] Joint Angles (rad) camera bot: [{joints_string_cam}]")


