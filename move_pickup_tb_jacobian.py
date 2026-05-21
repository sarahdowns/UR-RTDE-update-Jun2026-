# Author: Sarah Downs
# THIRD
# Detect an object and pick it up

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
from modern_robotics import IKinSpace, FKinSpace, ScrewTrajectory, MatrixLog6, TransInv, RpToTrans, MatrixExp6
#from ur_rtde import RTDEControlInterface, Path, PathEntry

# --- Configuration ---
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
    
# --- Check Jacobian ---

def ur_pose_to_transformation(pose):
    """Converts UR pose (xyz + RPY) to transformation matrix."""
    x, y, z, rx, ry, rz = pose
    theta = np.linalg.norm([rx, ry, rz])
    if theta < 1e-6:
        R = np.eye(3)
    else:
        r = np.array([rx, ry, rz]) / theta
        R = MatrixExp6(np.vstack((np.hstack((np.zeros((3, 3)), np.zeros((3, 1)))), np.zeros((1, 4)))) + np.block([
            [np.zeros((3, 3)), np.reshape(r, (3, 1)) * theta],
            [0, 0, 0, 0]
        ]))[:3, :3]
    T = RpToTrans(R, np.array([x, y, z]))
    return T

# IK solver using Modern Robotics
def my_ik_solver(T_target):
    q_guess = rtde_r.getActualQ()  # Seed with current joint angles
    ik_solutions = ur_kinematics.ik_all(T_target)  # Return list of all IK solutions

    if not ik_solutions:
        print("No IK solutions found for target pose:")
        print(T_target)
        return []

    return ik_solutions

# Check for singularities
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


# --- Transform to robot frame ---
ball_pos_robot = T_cam_to_robot @ ball_pos_cam
x, y, z = ball_pos_robot[:3]
print(f"Ball detected at robot frame: {x:.3f}, {y:.3f}, {z:.3f}")
input("Press Enter to move the robot to the ball...")

target_force = 30.00
rg_gripper.rg_grip(65, target_force)
print("Starting width: ",rg_width)
time.sleep(.5)

print("RTDE is connected:", rtde_c.isConnected())
print("Robot is in protective stop:", rtde_r.isProtectiveStopped())
print("Robot is in emergency stop:", rtde_r.isEmergencyStopped())

#rtde_c.moveJ([-1.0, -1.5, -2.0, -1.643, 2.679, 0.014])
#rtde_c.disconnect()

path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.156, 0.1579, 0.4248, -1.295, -0.198, -0.895, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.202, 0.159, 0.655, -1.317, -0.167, -0.895, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [-1.0, -1.5, -2.0, -1.643, 2.679, 0.014, 1.0, acc, 0.0]))  
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [0.18, -0.7977, 0.5287, 0.2652, 2.2483, -0.9034, vel, acc, 0.0]))

### Pose 5: Move to ball
# Convert the ball TCP pose to transformation matrix
tcp_pose_to_ball = [x, y, z, math.pi, 0, 0]
T_pose = ur_pose_to_transformation(tcp_pose_to_ball)  # tcp_pose_to_ball = [x, y, z, rx, ry, rz]

# Get all IK solutions for this pose
all_solutions = modern_robotics_IKinSpace_all(S, M, T_pose)

if not all_solutions:
    print("No IK solutions available for ball pose. Aborting.")
    rtde_c.stopScript()
    rtde_c.disconnect()
    exit()

# Filter elbow-up solutions
elbow_up_solutions = filter_elbow_up_solutions(all_solutions)

# Pick the best based on current joint config
current_q = rtde_r.getActualQ()
if elbow_up_solutions:
    chosen_q = select_best_solution(elbow_up_solutions, current_q)
else:
    print("No elbow-up solution found, falling back to closest available.")
    chosen_q = select_best_solution(all_solutions, current_q)

print("Moving to selected ball pose (joint angles):", chosen_q)

# maybe use Qnear command here?
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [*tcp_pose_to_ball, 0.2, 1.0, 0.0]))

# Send full path
success = rtde_c.movePath(path)
print("MovePath successful:", success)

# Cleanup
rtde_c.disconnect()
rtde_c.stopScript()

