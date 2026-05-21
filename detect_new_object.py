# meters

import pyzed.sl as sl
import cv2
import torch
import numpy as np
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', trust_repo=True)
model.conf = 0.5  # confidence threshold

# Init ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Better depth mode
init_params.coordinate_units = sl.UNIT.METER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("ZED camera failed to open.")
    exit(1)

runtime_params = sl.RuntimeParameters()
image_zed = sl.Mat()
point_cloud_zed = sl.Mat()

# Capture initial background frame
print("Capturing initial background frame for differencing...")
while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        initial_frame = image_zed.get_data()
        initial_gray = cv2.cvtColor(initial_frame, cv2.COLOR_RGBA2GRAY)
        initial_gray = cv2.GaussianBlur(initial_gray, (7, 7), 0)
        break

print("Running detection. Press ESC to quit.")

try:
    while True:
        if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud_zed, sl.MEASURE.XYZRGBA)

        frame = image_zed.get_data()  # RGBA
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # Process frame for difference mask
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
        frame_gray = cv2.GaussianBlur(frame_gray, (7, 7), 0)

        diff = cv2.absdiff(initial_gray, frame_gray)
        _, diff_thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

        # Morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        diff_thresh = cv2.morphologyEx(diff_thresh, cv2.MORPH_OPEN, kernel)
        diff_thresh = cv2.morphologyEx(diff_thresh, cv2.MORPH_DILATE, kernel)

        # Find contours on difference mask (new/moved objects)
        contours, _ = cv2.findContours(diff_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        # Run YOLOv5 detection
        results = model(frame_rgb)
        boxes = results.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2, conf, class]

        # Create mask from diff contours
        diff_mask = np.zeros(frame_gray.shape, dtype=np.uint8)
        cv2.drawContours(diff_mask, contours, -1, 255, thickness=cv2.FILLED)

        for box in boxes:
            x1, y1, x2, y2, conf, cls = box.astype(int)
            label = results.names[int(cls)]

            # Skip persons
            if label.lower() == "person":
                continue

            # Check if box center is in motion area
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            if diff_mask[cy, cx] == 0:
                continue

            # Find contour in bounding box
            roi_mask = diff_mask[y1:y2, x1:x2]
            contours_roi, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours_roi:
                largest_contour = max(contours_roi, key=cv2.contourArea)
                if len(largest_contour) >= 5:
                    ellipse = cv2.fitEllipse(largest_contour)
                    center, axes, angle = ellipse
                    center = (center[0] + x1, center[1] + y1)

                    # Draw ellipse
                    cv2.ellipse(frame_rgb, (center, axes, angle), (0, 255, 0), 2)

                    # Axes drawing
                    a = axes[0] / 2
                    b = axes[1] / 2
                    theta = np.radians(angle)
                    x0, y0 = center

                    # Major axis
                    x1_major = int(x0 + a * np.cos(theta))
                    y1_major = int(y0 + a * np.sin(theta))
                    x2_major = int(x0 - a * np.cos(theta))
                    y2_major = int(y0 - a * np.sin(theta))
                    cv2.line(frame_rgb, (x1_major, y1_major), (x2_major, y2_major), (255, 0, 0), 2)

                    # Minor axis
                    x1_minor = int(x0 + b * np.cos(theta + np.pi / 2))
                    y1_minor = int(y0 + b * np.sin(theta + np.pi / 2))
                    x2_minor = int(x0 - b * np.cos(theta + np.pi / 2))
                    y2_minor = int(y0 - b * np.sin(theta + np.pi / 2))
                    cv2.line(frame_rgb, (x1_minor, y1_minor), (x2_minor, y2_minor), (0, 255, 255), 2)

                    cv2.putText(frame_rgb, f"{label} Angle:{angle:.1f}°", (int(x0), int(y0 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # Get 3D position from point cloud
                    px, py = int(x0), int(y0)
                    point_cloud = point_cloud_zed.get_data()
                    if 0 <= py < point_cloud.shape[0] and 0 <= px < point_cloud.shape[1]:
                        point3D = point_cloud[py, px]
                        if not np.any(np.isnan(point3D)) and not np.any(np.isinf(point3D)):
                            X, Y, Z = point3D[:3]
                            pos_str = f"X:{X:.2f}m Y:{Y:.2f}m Z:{Z:.2f}m"
                            cv2.putText(frame_rgb, pos_str, (int(x0), int(y0 + 20)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            print(f"{label}: {X:.3f}, {Y:.3f}, {Z:.3f}")

        cv2.imshow("ZED + YOLOv5 New Object Ellipse Detection", frame_rgb)
        cv2.imshow("New Object Mask", diff_thresh)

        if cv2.waitKey(1) & 0xFF == 27:
            break
          

except KeyboardInterrupt:
    print("Interrupted by user.")

zed.close()
cv2.destroyAllWindows()
