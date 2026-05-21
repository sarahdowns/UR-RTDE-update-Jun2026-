# Author: Sarah Downs
# Move robot arms to a position above the table and receive coordinates from the 
# zed camera, pick up object. 

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2
import time

rg_id = 0
ip = "192.168.5.4"              # Change to desired robot IP
rg_gripper = RG2(ip,rg_id)

rtde_c = RTDEControl(ip)           
rtde_r = RTDEReceive(ip)

rg_width = rg_gripper.get_rg_width()
print("Starting width: ",rg_width)

init_q = rtde_r.getActualQ()
cam_posX = 0.3
cam_posY = -0.2
cam_posZ = 0.5
# Base on previous move orientaiton
cam_rx = -0.1015
cam_ry = -2.4698
cam_rz = 1.0424

path = Path()
vel = 0.5
acc = 2.0
blend = 0.099

## Floyd Positions
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.1008, 0.186, 0.376, -1.401, 0.406, 0.746, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.2016, 0.196, 0.644, -1.401, 0.406, 0.746, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [1.0, -2.0, 2.0, -1.5263, -2.8569, 0.167, vel, acc, blend]))                   # Joint position!!!
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.095, -0.8116, 0.497, -0.1015, -2.4698, 1.0424, vel, acc, blend]))         # TCP pose above table. Waiting
print("Awaiting camera position command...")

# Read the camera data
### STEP 1: Move to camera position ###
pre_grip_path = path  # Use your existing `path` object
pre_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [cam_posX, cam_posY, cam_posZ, cam_rx, cam_ry, cam_rz, vel, acc, 0.0]))

print("Moving to camera position...")
rtde_c.movePath(pre_grip_path, True)

# Wait until robot reaches position
while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.1)

### STEP 2: Perform the gripping action ###
target_force = 30.00
print("Reached camera position. Gripping object...")
rg_gripper.rg_grip(100, target_force)
time.sleep(2)
rg_gripper.rg_grip(50, target_force)

### STEP 3: Continue with rest of the movement ###
post_grip_path = Path()
post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [1.0, -2.0, 2.0, -1.5263, -2.8569, 0.167, vel, acc, blend]))

print("Lifting object...")
rtde_c.movePath(post_grip_path, True)

# Does a weird in between when commented out. But doesn't work when active
# while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
#     time.sleep(0.2)

# ## Poole Positions
# path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.156, 0.1579, 0.4248, -1.295, -0.198, -0.895, vel, acc, 0.0]))
# path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.202, 0.159, 0.655, -1.317, -0.167, -0.853, vel, acc, 0.0]))
## path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-0.0034, -3.0498, 2.2443, -1.643, 2.679, 0.01374, vel, acc, blend]))        # Joint position!!!
# path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.0, -1.5, -2.0, -1.643, 2.679, 0.01374, vel, acc, blend]))  
# path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.1823, -0.7846, 0.531, 0.2652, 2.2483, -0.9034, vel, acc, 0.0]))
# # Read the camera data

# path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.0, -1.5, -2.0, -1.643, 2.679, 0.01374, vel, acc, blend]))  

######## PATH MOVEMENT ################################
# First move given path synchronously
# print("Move path synchronously...")
# rtde_c.movePath(path, False)
# print("Path finished!")

# # Runs path function again
# print("Move path asynchronously with progress feedback...")
# rtde_c.movePath(path, True)

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