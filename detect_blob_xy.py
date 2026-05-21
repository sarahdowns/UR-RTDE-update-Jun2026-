import pyzed.sl as sl
import cv2
import numpy as np

def run_zed2i_object_axis_detection(display=True, area_threshold=800):
    """
    Starts a live feed from the ZED2i and performs object detection using contours.
    Calculates first and second moments (centroid, orientation), draws ellipses and principal axes.
    
    Args:
        display (bool): Whether to show OpenCV windows.
        area_threshold (int): Minimum area for a blob to be considered valid.
    """

    # --- ZED Camera Initialization ---
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED camera failed to open.")
        return

    cam_info = zed.get_camera_information()
    width = cam_info.camera_configuration.resolution.width
    height = cam_info.camera_configuration.resolution.height

    image_zed = sl.Mat(width, height, sl.MAT_TYPE.U8_C4)
    point_cloud_zed = sl.Mat(width, height, sl.MAT_TYPE.F32_C4)

    print("ZED2i Object Axis Detection Running. Press 'q' to quit.")

    try:
        while True:
            if zed.grab() != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud_zed, sl.MEASURE.XYZRGBA)

            frame = image_zed.get_data()
            point_cloud = point_cloud_zed.get_data()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            _, thresh = cv2.adaptiveThreshold(blurred, 255,
                               cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY_INV, 11, 2)


            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < area_threshold:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                if len(cnt) >= 5:
                    ellipse = cv2.fitEllipse(cnt)
                    center, axes, angle = ellipse
                    cv2.ellipse(frame, ellipse, (0, 255, 0), 2)

                    a = axes[0] / 2
                    b = axes[1] / 2
                    theta = np.radians(angle)
                    x0, y0 = center

                    # Major axis
                    x1 = int(x0 + a * np.cos(theta))
                    y1 = int(y0 + a * np.sin(theta))
                    x2 = int(x0 - a * np.cos(theta))
                    y2 = int(y0 - a * np.sin(theta))
                    cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Major = blue

                    # Minor axis
                    x3 = int(x0 + b * np.cos(theta + np.pi / 2))
                    y3 = int(y0 + b * np.sin(theta + np.pi / 2))
                    x4 = int(x0 - b * np.cos(theta + np.pi / 2))
                    y4 = int(y0 - b * np.sin(theta + np.pi / 2))
                    cv2.line(frame, (x3, y3), (x4, y4), (0, 255, 255), 2)  # Minor = yellow

                    cv2.putText(frame, f"Angle: {angle:.1f}°", (cx + 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)

                # 3D from point cloud
                if 0 <= cy < point_cloud.shape[0] and 0 <= cx < point_cloud.shape[1]:
                    point = point_cloud[cy, cx]
                    if not np.any(np.isnan(point)) and not np.any(np.isinf(point)):
                        x3d, y3d, z3d = point[:3]
                        cv2.putText(frame, f"X: {x3d:.2f}m", (cx + 10, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (255, 255, 255), 1)
                        cv2.putText(frame, f"Y: {y3d:.2f}m", (cx + 10, cy + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (255, 255, 255), 1)
                        cv2.putText(frame, f"Z: {z3d:.2f}m", (cx + 10, cy + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (255, 255, 255), 1)

            # Show output windows
            if display:
                cv2.imshow("ZED2i - Object Axis Detection", frame)
                cv2.imshow("Threshold Mask", thresh)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                # Headless mode: optionally log data here
                pass

    except KeyboardInterrupt:
        print("Interrupted by user.")

    zed.close()
    cv2.destroyAllWindows()

