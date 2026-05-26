import cv2
import numpy as np
import pyzed.sl as sl
import time
import os
from spatialmath import SE3

#DO NOT RUN -----------------------------------------------------------------

# Import your custom safe controller and local gripper driver
from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2

# --- User Configuration Block ---
ip = "192.168.5.5"
rg_id = 0
CALIB_FILE = "cam_to_robot_transform.npy"
GRIPPER_Z_LIMIT = -0.6  # Hard safety limit: Robot tool is blocked from going below Z = -0.6m

# Set gripper approach pitch (90 degrees keeps the gripper horizontal to the table)
PITCH_DEG = 90  

def load_calibration_matrix(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Calibration matrix '{filepath}' not found at {filepath}!")
    return np.load(filepath)

# --- 1. Your Custom Tennis Ball Detection Function ---
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

# --- 2. Robust Depth Point Filter ---
def get_robust_depth(point_cloud, cx, cy, window_size=5):
    """Samples a neighborhood around the centroid to filter out ZED point noise."""
    half_w = window_size // 2
    points = []
    for dy in range(-half_w, half_w + 1):
        for dx in range(-half_w, half_w + 1):
            err, pt = point_cloud.get_value(cx + dx, cy + dy)
            if err == sl.ERROR_CODE.SUCCESS:
                if not np.isnan(pt[0]) and not np.isnan(pt[1]) and not np.isnan(pt[2]):
                    points.append(pt[:3])
    if len(points) > 0:
        return np.median(points, axis=0)
    return None

# --- 3. Unified Coordinate Transformation Pipeline ---
def transform_cam_to_world_frame(p_cam, T_cam_to_robot, T_world_base):
    """
    Transforms coordinates from ZED Camera frame -> Raw Robot Base,
    then projects them into World/Table coordinates (matching the +45 deg base tilt).
    """
    # Step 1: Camera to Robot Base Frame (SVD Transform Matrix)
    p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
    p_base = T_cam_to_robot @ p_cam_h
    
    # Step 2: Robot Base to World Frame
    # T_world_base.A extracts the 4x4 numpy array representation of the SE3 base transform
    p_world = T_world_base.A @ np.array([p_base[0], p_base[1], p_base[2], 1.0])
    
    return p_world[:3]


def run_interactive_picker():
    # Initialize safety variables as None to protect 'finally' block from NameErrors
    arm = None
    zed = None

    try:
        # Load calibration matrix
        T_cam_to_robot = load_calibration_matrix(CALIB_FILE)
        print(f"[INFO] Hand-Eye Calibration Matrix Loaded.")

        # Pre-configure World frame matrix (Matching positive +45 deg base tilt in move_xyz_safe)
        T_world_base = SE3.Ry(np.deg2rad(45))

        # Initialize Safe Robot Controller
        print("[INIT] Connecting to UR5e Robot...")
        arm = UR5eSafeController(ip)
        
        # Initialize the OnRobot RG2 Driver (acting on the physical RG6)
        print("[INIT] Connecting to OnRobot RG Gripper...")
        rg_gripper = RG2(ip, rg_id)
        print("[GRIPPER] Gripper Connected successfully.")

        # Initialize ZED Camera
        print("[INIT] Starting ZED (Neural Depth Mode)...")
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        zed = sl.Camera()

        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("[ERROR] ZED camera failed to open.")
            return

        runtime_params = sl.RuntimeParameters()
        image_zed = sl.Mat()
        point_cloud = sl.Mat()

        print("\n" + "="*55)
        print(" READY FOR PICK AND PLACE")
        print(" Displaying feed. Press 'p' in OpenCV window to execute pick.")
        print(" Press 'q' to safely shut down.")
        print("="*55 + "\n")

        # Set initial physical posture (Open gripper to 60mm)
        rg_gripper.rg_grip(60, 25.0)

        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

                # Prevent cvtColor crash by stripping alpha channel
                frame = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                cx, cy, contour = detect_tennis_ball_center(hsv)

                if cx is not None and cy is not None:
                    ball_coords = get_robust_depth(point_cloud, cx, cy, window_size=5)
                    
                    if ball_coords is not None:
                        # Run the dual-transformation math
                        world_coords = transform_cam_to_world_frame(ball_coords, T_cam_to_robot, T_world_base)
                        corrected_x, corrected_y, corrected_z = world_coords

                        # Overlay detection visual details
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                        cv2.putText(frame, f"World X: {corrected_x:.3f} Y: {corrected_y:.3f} Z: {corrected_z:.3f}", 
                                    (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(frame, "TARGET LOCKED - Press 'p' to Pick", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                        cv2.imshow("Production Picker", frame)
                        key = cv2.waitKey(1) & 0xFF

                        if key == ord('p'):
                            print(f"\n[USER INPUT] Triggering trajectory to X: {corrected_x:.3f}, Y: {corrected_y:.3f}, Z: {corrected_z:.3f}")
                            
                            # Apply Safety Z Limit Guard to target position
                            if corrected_z < GRIPPER_Z_LIMIT:
                                print(f"[GUARD] Active! Target Z ({corrected_z:.3f}m) is below limit ({GRIPPER_Z_LIMIT}m). Clamping target.")
                                corrected_z = GRIPPER_Z_LIMIT

                            # Capture starting joint state before trajectory
                            initial_home_q = arm.rtde_r.getActualQ()

                            # Define safe target offsets based on the target center
                            PICK_Z = corrected_z
                            SAFE_HOVER_Z = PICK_Z + 0.20  # Hover exactly 20 cm above ball center

                            # Ensure the hover target is also within bounds
                            if SAFE_HOVER_Z < GRIPPER_Z_LIMIT:
                                SAFE_HOVER_Z = GRIPPER_Z_LIMIT

                            try:
                                # Tar F/T Sensor prior to movement sequence
                                arm.zero_ft_sensor()
                                time.sleep(0.5)

                                # --- Trajectory Sequence ---
                                
                                # STEP 1: Move to Hover above the ball
                                print(f"Moving to Hover position (Z: {SAFE_HOVER_Z:.3f})...")
                                arm.move_to_xyz_safe(corrected_x, corrected_y, SAFE_HOVER_Z, 
                                                     visualize=False, pitch_deg=PITCH_DEG)

                                # Baseline force check while hovering static
                                baseline = arm.get_ft_sensor_data()
                                print(f"Baseline Force (Z): {baseline[2]:.2f} N")

                                # STEP 2: Open Gripper prior to descent
                                print("Opening gripper to 60mm...")
                                rg_gripper.rg_grip(60, 25.0)
                                time.sleep(1.0)  # Complete physical open

                                # STEP 3: Lower to the ball's center position
                                print(f"Lowering to ball's center (Z: {PICK_Z:.3f})...")
                                arm.move_to_xyz_safe(corrected_x, corrected_y, PICK_Z, 
                                                     visualize=False, pitch_deg=PITCH_DEG)

                                # STEP 4: Close Gripper to secure target
                                print("Closing gripper to secure ball...")
                                rg_gripper.rg_grip(30, 25.0)
                                time.sleep(1.0)  # Complete physical squeeze

                                # Output landing contact measurements
                                final_ft = arm.get_ft_sensor_data()
                                contact_force_z = final_ft[2] - baseline[2]
                                total_force = np.linalg.norm(final_ft[:3])
                                print('-'*30)
                                print(f"Resulting Contact Force (Z): {contact_force_z:.2f} N")
                                print(f"Resulting Total Force Vector: {total_force:.2f} N")
                                print(f"Torques [Tx, Ty, Tz]: {np.round(final_ft[3:], 2)} Nm")
                                print('-'*30)

                                # STEP 5: Raise the gripper 0.2m
                                print("Raising gripper by 0.2m...")
                                arm.move_to_xyz_safe(corrected_x, corrected_y, PICK_Z + 0.20, 
                                                     visualize=False, pitch_deg=PITCH_DEG)
                                
                                # RETURN TO INITIAL POSITION OPTION
                                print("\n" + "="*30)
                                user_home = input("Task complete. Return to initial starting position? (y/n): ").lower()
                                if user_home == 'y':
                                    print("Returning to initial position...")
                                    arm.rtde_c.moveJ(initial_home_q, 0.2, 1.0)
                                    print("[STATUS] Returned to start.")
                                else:
                                    print("[INFO] Staying at current hover position.")

                            except Exception as path_error:
                                print(f"[PATH ERROR] Trajectory execution failed: {path_error}")

                        elif key == ord('q'):
                            break
                    else:
                        cv2.putText(frame, "Noisy depth at target.", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        cv2.imshow("Production Picker", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                else:
                    cv2.putText(frame, "Scanning for tennis ball...", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("Production Picker", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt caught.")
    finally:
        # Secure safety shutdown execution
        print("[CLEANUP] Terminating threads and sessions...")
        if zed is not None:
            zed.close()
        cv2.destroyAllWindows()
        if arm is not None:
            arm.reset_protective_stop()
            arm.cleanup()
        print("[STATUS] Shutdown clean. Arm connections severed.")

if __name__ == "__main__":
    run_interactive_picker()
