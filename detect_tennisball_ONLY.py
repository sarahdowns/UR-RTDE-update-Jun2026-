# Author: Sarah Downs
# File name: detect_tennisball_ONLY.py
# This script captures the tennis ball's 3D position (X, Y, Z) once every "interval" (seconds) for "duration" using a 
# ZED camera. The output is saved to a CSV file for later use.

import cv2
import numpy as np
import pyzed.sl as sl
import csv
import datetime
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


CAPTURE_INTERVAL = 1.0     # seconds
TOTAL_DURATION = 60.0     # seconds

def detect_tennis_ball_center(hsv_image):
    lower = np.array([38, 140, 140])
    upper = np.array([52, 255, 255])
    mask = cv2.inRange(hsv_image, lower, upper)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy, largest

    return None, None, None

def capture_tennis_ball_xyz():
    # Initialize ZED
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER

    zed = sl.Camera()
    ball_points = []
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Failed to open ZED camera.")
        exit(1)

    runtime_params = sl.RuntimeParameters()
    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"tb_camxyz_{timestamp}.csv"

    start_time = time.time()
    last_capture_time = 0

    print("[INFO] Capturing tennis ball XYZ data...")
    print("[INFO] Duration: 10 minutes | Interval: 1 second")

    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Ball_X", "Ball_Y", "Ball_Z"])


        while time.time() - start_time < TOTAL_DURATION:
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame_bgra = image_zed.get_data()
            frame = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            cx, cy, contour = detect_tennis_ball_center(hsv)

            current_time = time.time()
            if (
                cx is not None
                and cy is not None
                and current_time - last_capture_time >= CAPTURE_INTERVAL
            ):
                err, point = point_cloud.get_value(cx, cy)

                if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point[2]):
                    ball_x, ball_y, ball_z = point[:3]
                    writer.writerow([ball_x, ball_y, ball_z])
                    ball_points.append((ball_x, ball_y, ball_z))
                    last_capture_time = current_time

                    print(
                        f"[SAVED] X: {ball_x:.4f}, Y: {ball_y:.4f}, Z: {ball_z:.4f}"
                    )

                    # Visualization (optional)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

            cv2.putText(
                frame,
                "Recording tennis ball XYZ...",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.imshow("Tennis Ball Capture", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    zed.close()
    cv2.destroyAllWindows()
    print(f"[INFO] Finished. Data saved to: {filename}")
    
    # ==============================
    # Tennis Ball Detection Visualization
    # ==============================

    if len(ball_points) > 0:
        points = np.array(ball_points)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        mean_x, mean_y, mean_z = np.mean(points, axis=0)

        # --- 3D Scatter Plot ---
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")

        ax.scatter(x, y, z, s=10, alpha=0.6, label="Samples")
        ax.scatter(
            mean_x, mean_y, mean_z,
            color="green", s=60, label="Mean"
        )

        ax.set_title("Cam Scatter: Tennis Ball Position (Camera Frame)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()

        plt.tight_layout()
        plt.savefig("tb_camxyz.png", dpi=300)
        plt.close()


        # --- 2D Projections ---
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))

        axes[0].scatter(x, y, s=8, alpha=0.6)
        axes[0].set_xlabel("X (m)")
        axes[0].set_ylabel("Y (m)")
        axes[0].set_title("X vs Y")

        axes[1].scatter(x, z, s=8, alpha=0.6)
        axes[1].set_xlabel("X (m)")
        axes[1].set_ylabel("Z (m)")
        axes[1].set_title("X vs Z")

        axes[2].scatter(y, z, s=8, alpha=0.6)
        axes[2].set_xlabel("Y (m)")
        axes[2].set_ylabel("Z (m)")
        axes[2].set_title("Y vs Z")

        plt.tight_layout()
        plt.savefig("tb_camxyz_2D.png", dpi=300)
        plt.close()


    else:
        print("[WARNING] No valid tennis ball points collected. Skipping plot.")

if __name__ == "__main__":
    capture_tennis_ball_xyz()

