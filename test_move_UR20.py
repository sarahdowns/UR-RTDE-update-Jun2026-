# Author: Sarah Downs
# Move robot UR20

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2
import time

rg_id = 0
ip = "192.168.5.3"              # Change to UR20 IP
rg_gripper = RG2(ip,rg_id)

rtde_c = RTDEControl(ip)           
rtde_r = RTDEReceive(ip)
init_q = rtde_r.getActualQ()

path = Path()
vel = 0.3
acc = 2.0
blend = 0.099

## UR20 Positions
print("Moving UR20...")
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [1.4410, -1.6907, 2.8976, -1.6444, -3.1377, -0.5402, vel, acc, blend]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [1.5062, -1.3677, 2.4667, -1.1266, -3.5477, -0.1121, vel, acc, blend]))


print("Move path asynchronously with progress feedback...")
rtde_c.movePath(path, True)

# Wait for start of asynchronous operation
while not rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.010)
print("Async path started.. ")

# Wait for end of asynchronous operation
waypoint = -1
while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.2)
    new_waypoint = rtde_c.getAsyncOperationProgress()
    if new_waypoint != waypoint:
        waypoint = new_waypoint
        print("Moving to path waypoint ")

print("Async path finished...\n\n")
rtde_c.moveJ(init_q)                     # Return to intial position
print("Returned to initial position.")
rtde_c.stopScript()