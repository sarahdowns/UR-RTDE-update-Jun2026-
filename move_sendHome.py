# Author: Sarah Downs
# Release the gripper and send home from above the table

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

rg_width = rg_gripper.get_rg_width()
init_q = rtde_r.getActualQ()

path = Path()
vel = 0.5
acc = 2.0
blend = 0.099

# Starting Gripper width
target_force = 30.00
rg_gripper.rg_grip(50, target_force)
print("Starting width: ", rg_width)
time.sleep(.5)

path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-1.0354, -1.4877, -1.8264, -0.3177, 1.7051, 0.8535, 0.2, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-0.6777, -1.6844, -0.9829, -0.4449, 2.1427, 0.7461, vel, acc, 0.0])) 
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.1594, -2.5026, 0.1402, -0.1658, 2.5696, 0.0785, vel, acc, 0.0]))

return_path = Path()
return_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [0.0000, -3.6374, 2.8053, -1.5785, 2.6658, 0.0002, vel, acc, 0.0])) 

print("Starting return movements...")
rtde_c.movePath(return_path, True) # 'True' means synchronous execution

while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.05)
         
rtde_c.stopScript()


#### Note ####
# TCP Pose
# Out: [ , ,  ,  ,   , +]

# Joint Pose
# CW: [ , ,  ,  , - , +]