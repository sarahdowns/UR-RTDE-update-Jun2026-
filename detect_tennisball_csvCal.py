# Author: Sarah Downs
# FIRST
# This code will record the tennis ball's x, y, z and the UR robot's x, y, z, rx, ry, rz TCP pose at
# each saved point. Maintain the orientation of the gripper for every point. This code can be run in local mode so the the robot can be moved with free drive 
# to multiple points for calibration. The output is a csv file with these points. 

import cv2
import numpy as np
import pyzed.sl as sl
import csv
import os
import datetime
import time
from rtde_receive import RTDEReceiveInterface

# Initialize RTDE
rtde_r = RTDEReceiveInterface("192.168.5.5")  # Your robot IP

def detect_tennis_ball_center(hsv_image):
    lower = np.array([38, 140, 140])
    upper = np.array([52, 255, 255])
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

def write_to_csv(file, row):
    writer = csv.writer(file)
    writer.writerow(row)

def detect_tennis_ball_and_log():
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    zed = sl.Camera()

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Failed to open ZED camera.")
        exit(1)

    runtime_params = sl.RuntimeParameters()
    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"zed_ur_tennisball_{timestamp}.csv"

    print("[INFO] Press 's' to save a detected point, 'q' to quit.")

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Ball_X", "Ball_Y", "Ball_Z", "TCP_X", "TCP_Y", "TCP_Z", "TCP_Rx", "TCP_Ry", "TCP_Rz"])

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
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                        cv2.putText(frame, f"X: {ball_x:.3f} Y: {ball_y:.3f} Z: {ball_z:.3f}", (cx + 10, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.putText(frame, "Press 's' to save, 'q' to quit", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                        cv2.imshow("Tennis Ball Detection", frame)
                        key = cv2.waitKey(1) & 0xFF

                        if key == ord('s'):
                            time.sleep(0.5)
                            tcp_pose = rtde_r.getActualTCPPose()
                            tcp_x, tcp_y, tcp_z, rx, ry, rz = tcp_pose
                            write_to_csv(file, [ball_x, ball_y, ball_z, tcp_x, tcp_y, tcp_z, rx, ry, rz])
                            print(f"[SAVED] Ball at ({ball_x:.3f}, {ball_y:.3f}, {ball_z:.3f})")

                        elif key == ord('q'):
                            break
                    else:
                        print("[WARNING] Invalid depth at ball location.")
                else:
                    cv2.putText(frame, "No tennis ball detected", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("Tennis Ball Detection", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

    zed.close()
    cv2.destroyAllWindows()
    print(f"[INFO] Data saved to: {filename}")

if __name__ == "__main__":
    detect_tennis_ball_and_log()

