# Author: Sarah Downs
# THIRD, VERSION 3!
# DOESN'T WORK
# Addition: Asma's simple move code to avoid singularities. 
# The camera preview will appear and display if the tennisball is detected through a mask detection (green ring and red dot),
# once this looks correct press 'q' and confirm the ball's position. Then press enter to run the program and send the arm
# to the location of the ball. 

import cv2
import numpy as np
import pyzed.sl as sl
import csv
import datetime
import math
import time

from tennisball_detection import detect_ball_position_from_zed
from gripper_RG2 import RG2
from plot_TCP_path import visualize_tcp_path

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
# from modern_robotics import IKinSpace, FKinSpace, ScrewTrajectory, MatrixLog6, TransInv, RpToTrans, MatrixExp6
#from ur_rtde import RTDEControlInterface, Path, PathEntry

import modern_robotics as mr 

# Singularity check
from singularity_only_checking_working import correct_and_check_trajectory, plot_trajectories


# --- Configuration ---
rg_id = 0
ip = "192.168.5.5"              # Change to desired robot IP
rg_gripper = RG2(ip,rg_id)

rtde_c = RTDEControl(ip)           
rtde_r = RTDEReceive(ip)

rg_width = rg_gripper.get_rg_width()
init_q = rtde_r.getActualQ()

vel = 0.5
acc = 2.0
blend = 0.099

# --- Modern_robotics for the jacobian ---
M = np.array([				# Home configuration of the end-effector
    [  0.153,  0.952,  0.265,  0.156  ],
    [ -0.741,  0.289, -0.608,  0.1579 ],
    [ -0.654, -0.103,  0.750,  0.4248 ],
    [   0.0,     0.0,    0.0,    1.0  ]
])
S = np.array([				# Space-frame screw axes for UR5
    [ 0,  0,  0,  0,   1,   0],
    [ 0,  1,  1,  0,   0,   0],
    [ 1,  0,  0,  1,   0,   0],
    [ 0, -0.162, -0.425, -0.817, 0, 0],
    [ 0,  0,  0,  0,   0,   0],
    [ 0,  0,  0,  0,   0,   0]
])


# --- Load Transformation Matrix ---
try:
    T_cam_to_robot = np.load("cam_to_robot_transform.npy")  # 4x4 matrix from calibration
    print("Loaded transformation matrix:\n", T_cam_to_robot)
except FileNotFoundError:
    print("ERROR: cam_to_robot_transform.npy not found!")
    exit()

# --- Initialize ZED camera ---
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.NEURAL
init_params.coordinate_units = sl.UNIT.METER
zed = sl.Camera()
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("[ERROR] Failed to open ZED camera.")
    exit()

runtime_params = sl.RuntimeParameters()
image_zed = sl.Mat()
point_cloud = sl.Mat()

def detect_tennis_ball_center(hsv_image):
    lower = np.array([25, 70, 90])
    upper = np.array([45, 255, 255])
    mask = cv2.inRange(hsv_image, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return cx, cy, largest
    return None, None, None

# --- Detect tennis ball in camera frame ---
print("Detecting tennis ball... Press 'q' to quit.")

ball_pos_cam = None
while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        frame = image_zed.get_data()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cx, cy, contour = detect_tennis_ball_center(hsv)

        if cx is not None and cy is not None:
            err, point = point_cloud.get_value(cx, cy)
            if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point[2]):
                ball_x, ball_y, ball_z = point[:3]
                ball_pos_cam = np.array([ball_x, ball_y, ball_z, 1.0])  # homogeneous

                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.putText(frame, f"X: {ball_x:.3f} Y: {ball_y:.3f} Z: {ball_z:.3f}", (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, "Press 'q' to quit and move robot", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                cv2.putText(frame, "Invalid depth at ball location", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(frame, "No tennis ball detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Tennis Ball Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') and ball_pos_cam is not None:
            break
    else:
        print("[ERROR] ZED grab failed.")

zed.close()
cv2.destroyAllWindows()

if ball_pos_cam is None:
    print("No valid tennis ball detected, exiting.")
    exit()
    

# --- Pose helper
def ur_pose_to_transformation(pose):
    """Converts UR pose (xyz + axis-angle) to transformation matrix."""
    x, y, z, rx, ry, rz = pose
    r = np.array([rx, ry, rz])
    theta = np.linalg.norm(r)
    if theta < 1e-6:
        R = np.eye(3)
    else:
        w = r / theta
        R = mr.MatrixExp3(mr.VecToso3(w * theta))
    return mr.RpToTrans(R, np.array([x, y, z]))

'''
# IK solver using Modern Robotics
def my_ik_solver(T_target):
    q_guess = rtde_r.getActualQ()  # Seed with current joint angles
    ik_solutions = ur_kinematics.ik_all(T_target)  # Return list of all IK solutions

    if not ik_solutions:
        print("No IK solutions found for target pose:")
        print(T_target)
        return []

    return ik_solutions

# Check for singularities from v2
def check_singularity_from_pose(rtde_r, pose, ik_solver):
    print("Pose to solve IK for:", pose)
    
    T_pose = ur_pose_to_transformation(pose)
    q_sol = ik_solver(T_pose)

    J = mr.JacobianSpace(S, q_sol)
    cond_number = np.linalg.cond(J)
    print(f"Jacobian condition number: {cond_number:.2f}")
    
    threshold = 100  # Typical threshold; tune this based on robot sensitivity
    return cond_number > threshold, cond_number
    
def my_ik_solver_all_solutions(T_target):
    """Returns all possible joint solutions for a desired end-effector pose using IKinSpace."""
    thetalist0 = np.zeros(6)
    eomg = 1e-3
    ev = 1e-3

    try:
        sol, success = IKinSpace(S, M, T_target, thetalist0, eomg, ev)
        if not success:
            return []
        return [sol]
    except Exception as e:
        print("IK solver failed:", e)
        return []

def filter_elbow_up_solutions(solutions):
    """Filters out 'elbow-down' joint configurations for UR5. Here, we assume joint 3 should be negative (elbow up)."""
    return [q for q in solutions if q[2] < 0]

def select_best_solution(solutions, current_q):
    """Selects the IK solution closest to the robot's current joint angles."""
    return min(solutions, key=lambda q: np.linalg.norm(np.array(q) - np.array(current_q)))
'''

# --- Transform to robot frame ---
ball_pos_robot = T_cam_to_robot @ ball_pos_cam
x, y, z = ball_pos_robot[:3]
print(f"Ball detected at robot frame: {x:.3f}, {y:.3f}, {z:.3f}")
distance = np.linalg.norm([x, y, z])
print(f"Distance from base origin: {distance:.3f} m")

# Reachability check for UR5
MAX_REACH = 0.85   # meters (UR5 max reach)
SAFE_REACH = 0.80  # meters (practical safe limit)

if distance > MAX_REACH:
    print("Target is OUTSIDE UR5 maximum reach. IK will fail.")
elif distance > SAFE_REACH:
    print("[INFO] Target is near the edge of reachability. IK may fail or be unstable.")
else:
    print("Target is within safe reach.")
    
print("==============================================\n")
input("Press Enter to move the robot to the ball...")

# Prepare gripper
target_force = 30.00
rg_gripper.rg_grip(65, target_force)
print("Starting width: ", rg_width)
time.sleep(.5)

# Check RTDE connection
print("RTDE is connected:", rtde_c.isConnected())
print("Robot is in protective stop:", rtde_r.isProtectiveStopped())
print("Robot is in emergency stop:", rtde_r.isEmergencyStopped())

# --- predefined approach waypoints ---
path1 = Path()

path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.156, 0.1579, 0.4248, -1.295, -0.198, -0.895, vel, acc, 0.0]))
path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.202, 0.159, 0.655, -1.317, -0.167, -0.895, vel, acc, 0.0]))
path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints,   [-1.0, -1.5, -2.0, -1.643, 2.679, 0.014, vel, acc, 0.0]))
path1.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.18, -0.7977, 0.5287, 0.2652, 2.2483, -0.9034, vel, acc, 0.0]))

print("Executing approach waypoints...")
ok = rtde_c.movePath(path1)
print("Phase 1 movePath successful:", ok)
if not ok:
    print("Approach path failed. Stopping.")
    rtde_c.stopScript()
    rtde_c.disconnect()
    raise SystemExit
    
time.sleep(0.5)

if not rtde_c.isConnected():
    print("RTDE control not connected. Reconnecting...")
    rtde_c.connect()
# --- Current robot state ---
current_q   = rtde_r.getActualQ()
current_tcp = rtde_r.getActualTCPPose()

ball_pos = np.array([x, y, z])

approach_point = ball_pos + np.array([0, 0, 0.15])  # 15cm above ball
rx, ry, rz = -1.57, 0, 0  # point straight down
approach_pose = [*approach_point, rx, ry, rz]
ball_pose = [*ball_pos, rx, ry, rz]

############################################################################3333
### Current point: Moves through path1 but does not approach ball. Was moving back instead of toward the ball before
### but now not moving at all.  

# --- Attempt approach ---
print("Moving to approach pose...")

ball_success = False
current_q = rtde_r.getActualQ()

try:
    success_approach = rtde_c.moveJ_IK(approach_pose, vel, acc)
except Exception as e:
    print("moveJ_IK failed:", e)
    success_approach = False

if success_approach:
    try:
        rtde_c.moveL(ball_pose, vel/2, acc/2)
        ball_success = True
    except Exception as e:
        print("moveL failed:", e)
        ball_success = False

# --- Fallback if internal IK fails ---
if not ball_success:
    print("[INFO] UR internal IK failed. Using Modern Robotics solver...")
    T_pose = ur_pose_to_transformation(ball_pose)
    thetalist0 = np.array(current_q)
    eomg, ev = 1e-3, 1e-3

    sol, success = mr.IKinSpace(S, M, T_pose, thetalist0, eomg, ev)
    if not success:
        print("No valid IK solution found. Aborting.")
        rtde_c.stopScript()
        rtde_c.disconnect()
        exit()

    chosen_q = sol
    path2 = Path()
    path2.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [*chosen_q, vel, acc, 0.0]))
    rtde_c.movePath(path2)
    print("Fallback path executed.")
else:
    print("Successfully moved to ball pose.")
 

# --- Filter elbow-up solutions ---
'''
elbow_up_solutions = filter_elbow_up_solutions(all_solutions)

# --- Pick the best solution based on current joint angles ---
current_q = rtde_r.getActualQ()
if elbow_up_solutions:
    chosen_q = select_best_solution(elbow_up_solutions, current_q)
else:
    print("No elbow-up solution found, falling back to closest available.")
    chosen_q = select_best_solution(all_solutions, current_q)

# --- Check trajectory for singularities ---
planned_path = [current_q, chosen_q]
is_singular, corrected_path, dh_params = correct_and_check_trajectory(planned_path)

if is_singular:
    print("⚠ Warning: Singularity detected in path. Using corrected joint angles.")
    chosen_q = corrected_path[-1]  # Use corrected final joint configuration
    # Optional: visualize the path
    plot_trajectories(planned_path, corrected_path, dh_params)

print("Moving to selected ball pose (joint angles):", chosen_q)

# --- Add final MoveJ to ball with TCP pose ---
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [*tcp_pose_to_ball, 0.2, 1.0, 0.0]))

# --- Send full path to robot ---
success = rtde_c.movePath(path)
print("MovePath successful:", success)
'''

# --- Cleanup ---
rtde_c.stopScript()
rtde_c.disconnect()
