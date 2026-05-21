# Author: Sarah Downs

import cv2
import numpy as np
import pyzed.sl as sl
import csv
import os
import datetime
import time
from rtde_receive import RTDEReceiveInterface

try:
    rtde_r = RTDEReceiveInterface("192.168.5.5")
    print("[INFO] Successfully connected to RTDE.")
except RuntimeError as e:
    print(f"[ERROR] Could not connect to RTDE: {e}")
    print("[WARNING] The script will run without RTDE logging functionality.")
    rtde_r = None

def detect_black_rod(hsv_image):
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 60])
    mask = cv2.inRange(hsv_image, lower, upper)

    # Apply morphological operations to reduce noise and close gaps
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
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

            # Calculate aspect ratio (height / width)
            aspect_ratio = height / width if width > 0 else 0
            
            # Filter based on a minimum aspect ratio to detect elongated objects
            # A 20cm x 3cm rod has an aspect ratio of 20/3 ~6.7.
            if aspect_ratio > 5: # Adjust this value to filter for your rod's shape
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    return cx, cy, contour
    return None, None, None

def write_to_csv(file, row):
    writer = csv.writer(file)
    writer.writerow(row)

def detect_black_rod_and_log():
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
    
    # Define a smaller region of data collection
    x_start, y_start = 300, 200
    x_end, y_end = 1100, 700
    
    # Setup for CSV logging
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"zed_ur_black_rod_{timestamp}.csv"
    
    print("[INFO] Press 's' to save a detected point, 'q' to quit.")
    print("[INFO] Looking for the rod within the reduced region.")

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Rod_X", "Rod_Y", "Rod_Z", "TCP_X", "TCP_Y", "TCP_Z", "TCP_Rx", "TCP_Ry", "TCP_Rz"])		# In the camera frame

        while True:
            # Grab a new frame from the ZED camera
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve the left image and point cloud
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

                frame = image_zed.get_data()
                
                # Draw the ROI rectangle on the full frame for visualization
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)
                
                # Crop the frame to the ROI for processing
                roi_frame = frame[y_start:y_end, x_start:x_end]
                hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

                # Detect the rod within the ROI
                cx_roi, cy_roi, contour = detect_black_rod(hsv)

                # Define text positions for the bottom-left corner
                text_y_offset = frame.shape[0] - 30  # Adjust as needed for spacing
                coord_text_pos = (10, text_y_offset)
                angle_text_pos = (10, text_y_offset - 30)

                if cx_roi is not None and cy_roi is not None:
                    # Translate the coordinates from ROI to the full image frame
                    cx = cx_roi + x_start
                    cy = cy_roi + y_start
                    
                    # Retrieve the 3D point from the point cloud at the detected pixel
                    err, point = point_cloud.get_value(cx, cy)
                    
                    if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point[2]):
                        rod_x, rod_y, rod_z = point[:3]
                        
                        # Get the rotated bounding box for the contour
                        rect = cv2.minAreaRect(contour)
                        box = np.intp(cv2.boxPoints(rect))

                        # Calculate the center of the bounding box and its angle
                        rect_center = rect[0]
                        rect_angle = rect[2]
                        width, height = rect[1]

                        # Correct the angle to a 0-180 degree range
                        if width < height:
                            angle = rect_angle + 90
                        else:
                            angle = rect_angle
                        
                        if angle < 0:
                            angle += 180

                        # Calculate the endpoints of the center line
                        line_length = max(width, height) / 2
                        line_x1 = int(rect_center[0] - line_length * np.cos(np.radians(angle)))
                        line_y1 = int(rect_center[1] - line_length * np.sin(np.radians(angle)))
                        line_x2 = int(rect_center[0] + line_length * np.cos(np.radians(angle)))
                        line_y2 = int(rect_center[1] + line_length * np.sin(np.radians(angle)))
                        
                        # Translate coordinates from ROI to full frame
                        line_x1 += x_start
                        line_y1 += y_start
                        line_x2 += x_start
                        line_y2 += y_start
                        
                        # Draw the center line
                        cv2.line(frame, (line_x1, line_y1), (line_x2, line_y2), (0, 255, 0), 2)
                        
                        # Draw the bounding box on the full frame
                        box[:, 0] += x_start
                        box[:, 1] += y_start
                        cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                        # Display the coordinates and angle in the bottom-left corner
                        cv2.putText(frame, f"X: {rod_x:.3f} Y: {rod_y:.3f} Z: {rod_z:.3f}", coord_text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.putText(frame, f"Angle: {angle:.2f} deg", angle_text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.putText(frame, "Press 's' to save, 'q' to quit", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        cv2.imshow("Black Rod Detection", frame)
                        key = cv2.waitKey(1) & 0xFF

                        if key == ord('s'):
                            if rtde_r:
                                tcp_pose = rtde_r.getActualTCPPose()
                                tcp_x, tcp_y, tcp_z, rx, ry, rz = tcp_pose
                                write_to_csv(file, [rod_x, rod_y, rod_z, tcp_x, tcp_y, tcp_z, rx, ry, rz])
                                print(f"[SAVED] Rod at ({rod_x:.3f}, {rod_y:.3f}, {rod_z:.3f})")
                            else:
                                print("[WARNING] Cannot save: RTDE connection not established.")

                        elif key == ord('q'):
                            break
                    else:
                        print("[WARNING] Invalid depth at rod location.")
                        cv2.imshow("Black Rod Detection", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                else:
                    cv2.putText(frame, "No black rod detected", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("Black Rod Detection", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
    # Clean up and close resources
    zed.close()
    cv2.destroyAllWindows()
    print(f"[INFO] Data saved to: {filename}")

if __name__ == "__main__":
    detect_black_rod_and_log()
