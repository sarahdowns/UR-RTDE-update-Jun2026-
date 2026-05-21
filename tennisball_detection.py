# tennisball_detection.py

import cv2
import numpy as np
import pyzed.sl as sl
import time

def detect_ball_position_from_zed(display=False):
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

    # ZED camera setup
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    zed = sl.Camera()

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Failed to open ZED camera.")
        return None

    runtime_params = sl.RuntimeParameters()
    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    ball_position = None
    timeout = time.time() + 20  # Try for up to 20 seconds

    while time.time() < timeout:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame = image_zed.get_data()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            cx, cy, contour = detect_tennis_ball_center(hsv)

            if cx is not None and cy is not None:
                err, point = point_cloud.get_value(cx, cy)
                if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point[2]):
                    ball_position = point[:3]  # X, Y, Z

                    if display:
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                        cv2.putText(frame, f"X: {point[0]:.3f} Y: {point[1]:.3f} Z: {point[2]:.3f}", (cx + 10, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.imshow("Ball Detection", frame)
                        cv2.waitKey(500)

                    break  # Found valid ball position
        else:
            time.sleep(0.05)

    zed.close()
    if display:
        cv2.destroyAllWindows()

    return ball_position  # or None

