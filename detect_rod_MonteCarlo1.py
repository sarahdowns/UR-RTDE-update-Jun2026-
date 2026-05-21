# Author: Sarah Downs
# This script is for collecting data for a Monte Carlo simulation
# to show how much a ZED2i camera's measurements can vary.

import cv2
import numpy as np
import pyzed.sl as sl
import csv
import datetime
import time
from rtde_receive import RTDEReceiveInterface

# Note: The RTDE interface is included but will not be used in this script.
# It's kept for potential future use or for a separate logging script.
try:
    rtde_r = RTDEReceiveInterface("192.168.5.5")
    print("[INFO] Successfully connected to RTDE.")
except RuntimeError as e:
    print(f"[ERROR] Could not connect to RTDE: {e}")
    print("[WARNING] The script will proceed without RTDE functionality.")
    rtde_r = None

def detect_black_rod(hsv_image):
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 60])
    mask = cv2.inRange(hsv_image, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in sorted_contours:
            area = cv2.contourArea(contour)
            if area < 500:
                continue

            rect = cv2.minAreaRect(contour)
            width, height = rect[1]
            
            if width > height:
                width, height = height, width

            aspect_ratio = height / width if width > 0 else 0
            
            if aspect_ratio > 5:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    return cx, cy, contour
    return None, None, None

def write_to_csv(file, row):
    """
    Helper function to write a row to a CSV file.
    """
    writer = csv.writer(file)
    writer.writerow(row)

def detect_black_rod_and_log():
    """
    Initializes the ZED camera, detects the black rod, and logs data.
    """
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
    
    # Define the Region of Interest (ROI)
    x_start, y_start = 300, 200
    x_end, y_end = 1100, 700
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"zed_ur_black_rod_3000_frames_{timestamp}.csv"
    
    print(f"[INFO] Capturing 3000 frames. Data will be saved to: {filename}")

    frame_count = 0
    max_frames = 30000

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Rod_X", "Rod_Y", "Rod_Z"])

        while frame_count < max_frames:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
                
                frame = image_zed.get_data()
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)
                
                roi_frame = frame[y_start:y_end, x_start:x_end]
                hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
                cx_roi, cy_roi, contour = detect_black_rod(hsv)

                if cx_roi is not None and cy_roi is not None:
                    cx = cx_roi + x_start
                    cy = cy_roi + y_start
                    err, point = point_cloud.get_value(cx, cy)
                    
                    if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point[2]):
                        rod_x, rod_y, rod_z = point[:3]
                        write_to_csv(file, [rod_x, rod_y, rod_z])
                        frame_count += 1
                        
                        # Visualization (optional)
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                        rect = cv2.minAreaRect(contour)
                        box = np.intp(cv2.boxPoints(rect))
                        box[:, 0] += x_start
                        box[:, 1] += y_start
                        cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                        
                        cv2.putText(frame, "Capturing frames...", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        print(f"[INFO] Captured frame {frame_count}/{max_frames}")
                    
                cv2.imshow("Black Rod Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    zed.close()
    cv2.destroyAllWindows()
    print(f"[INFO] Data collection complete. Data saved to: {filename}")

if __name__ == "__main__":
    detect_black_rod_and_log()
