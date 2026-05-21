import numpy as np
import pyzed.sl as sl
import time
from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ================= ROBOT SETUP =================
ROBOT_IP = "192.168.5.5"  # Your UR5 IP
rtde_r = RTDEReceiveInterface(ROBOT_IP)
rtde_c = RTDEControlInterface(ROBOT_IP)

# ================= SAFETY LIMITS =================
workspace_limits = {
    'x': [0.2, 0.8],
    'y': [-0.4, 0.4],
    'z': [0.1, 0.6]
}

# ================= SMOOTHING =================
last_pos = None
alpha = 0.3  # exponential smoothing factor

# ================= BALL DETECTION =================
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
            return cx, cy
    return None, None

def clip_to_workspace(x, y, z):
    x = np.clip(x, *workspace_limits['x'])
    y = np.clip(y, *workspace_limits['y'])
    z = np.clip(z, *workspace_limits['z'])
    return x, y, z

# ================= HEADLESS TRACKING =================
def track_ball_real_time_headless():
    global last_pos
    import cv2  # only for color conversions
    # ZED camera setup
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    zed = sl.Camera()
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Failed to open ZED camera.")
        return

    runtime_params = sl.RuntimeParameters()
    image_zed = sl.Mat()
    point_cloud = sl.Mat()

    # ================= SETUP LIVE 3D PLOT =================
    plt.ion()
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(workspace_limits['x'])
    ax.set_ylim(workspace_limits['y'])
    ax.set_zlim(workspace_limits['z'])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("UR5 TCP and Tennis Ball Tracking")
    print("[INFO] Press Ctrl+C to exit.")

    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
                frame = image_zed.get_data()
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                cx, cy = detect_tennis_ball_center(hsv)

                if cx is not None and cy is not None:
                    err, point = point_cloud.get_value(cx, cy)
                    if err == sl.ERROR_CODE.SUCCESS and not np.isnan(point[2]):
                        ball_x, ball_y, ball_z = point[:3]

                        # Smooth motion
                        if last_pos is None:
                            last_pos = np.array([ball_x, ball_y, ball_z])
                        smoothed = alpha * np.array([ball_x, ball_y, ball_z]) + (1 - alpha) * last_pos
                        last_pos = smoothed

                        # Clip to workspace
                        x, y, z = clip_to_workspace(*smoothed)

                        # Move robot (fixed orientation)
                        q = [0, 1, 0, 0]
                        rtde_c.servoL([x, y, z, q[0], q[1], q[2], q[3]], 0.5, 0.5, 0.01, 0.0, 0.0)

                        # Get TCP pose for plotting
                        tcp_pose = rtde_r.getActualTCPPose()

                        # Update 3D plot
                        ax.cla()
                        ax.set_xlim(workspace_limits['x'])
                        ax.set_ylim(workspace_limits['y'])
                        ax.set_zlim(workspace_limits['z'])
                        ax.set_xlabel("X (m)")
                        ax.set_ylabel("Y (m)")
                        ax.set_zlabel("Z (m)")
                        ax.set_title("UR5 TCP and Tennis Ball Tracking")
                        ax.scatter(ball_x, ball_y, ball_z, c='r', label='Ball')
                        ax.scatter(tcp_pose[0], tcp_pose[1], tcp_pose[2], c='b', label='TCP')
                        ax.legend()
                        plt.draw()
                        plt.pause(0.001)
    except KeyboardInterrupt:
        print("[INFO] Tracking stopped by user.")
    finally:
        zed.close()
        plt.ioff()
        print("[INFO] Exiting tracking.")

if __name__ == "__main__":
    track_ball_real_time_headless()

