# Author: Sarah Downs
# File name: move_insertion_algorithm.py
# This script executes a blind tactile search. It auto-calibrates the rod length 
# on the first plunge, then uses that measurement for subsequent safe hovers and searches.

import time
import math
import numpy as np
from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2

ip = "192.168.5.5"
rg_id = 0

GRIPPER_OPEN = 60.0         # mm
GRIPPER_CLOSED = 15.0       # mm
GRIPPER_FORCE = 40.0        # N
GRIPPER_COMPLIANCE = 20.5     # mm
GRIPPER_COMP_FORCE = 20.0   # N

# --- Kinematic & Orientation Parameters ---
ROLL_DEG = -90
PITCH_DEG = 0
YAW_DEG = 90
DESCENT_SPEED = -0.05  # m/s (5 cm/s downwards)
LATERAL_SPEED = -0.03  # m/s (3 cm/s in the -X World Direction)
FORCE_NEG_Z_LIMIT = 12.0
FORCE_LATERAL_LIMIT = 8.0 # Stop translating when hitting a fixture/wall
SOCKET_SURFACE_Z = .122
SAFETY_OFFSET = 0.05

# --- Environmental Ground Truths ---
TABLE_TOP_Z = -0.02  # Hardcoded table height in the 45-deg World Frame
SAFE_APPROACH_Z = 0.3  # Absolute Gripper Z-height to use BEFORE we know the rod length
hole_diameter = .03 # meters
# Position 1 (Initial Starting Point)
START_X = 0.15
START_Y = -0.6

Socket_start_X = -0.07
Socket_start_Y = -0.6

Socket_left_X = -0.16
Socket_left_Y = -0.6

# Jun 9th socket at
#World X :  -0.0983 m
#World Y :  -0.6000 m
#World Z :   0.1223 m


def plunge_until_contact(robot):
    """
    Drives down until force limit is hit. 
    Returns the absolute World Z-coordinate of the GRIPPER at the moment of impact.
    """
    robot.zero_ft_sensor()
    time.sleep(0.5)
    
    baseline_ft = robot.get_ft_sensor_baserot(world_frame=True)
    baseline_f_z = baseline_ft[2]
    
    print(f"[SEARCH] Plunging at {abs(DESCENT_SPEED)*100} cm/s. Baseline Z-Force: {baseline_f_z:.2f} N")

    theta = np.deg2rad(45)
    R_world_base = np.array([
        [np.cos(theta),  0.0, np.sin(theta)],
        [0.0,            1.0, 0.0          ],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])
    
    V_world = np.array([0.0, 0.0, DESCENT_SPEED])
    V_base = R_world_base.T @ V_world 
    speedl_command = [V_base[0], V_base[1], V_base[2], 0.0, 0.0, 0.0]
    
    while True:
        current_ft = robot.get_ft_sensor_baserot(world_frame=True)
        delta_force_z = abs(current_ft[2] - baseline_f_z)
        
        if delta_force_z > FORCE_NEG_Z_LIMIT:
            robot.rtde_c.speedStop() # INSTANT BRAKE
            
            # Capture the exact pose of the TCP
            current_pose = robot.rtde_r.getActualTCPPose()
            p_base_h = np.array([current_pose[0], current_pose[1], current_pose[2], 1.0])
            
            T_world_base = np.eye(4)
            T_world_base[:3, :3] = R_world_base
            p_world_h = T_world_base @ p_base_h
            
            gripper_world_z = p_world_h[2]
            
            print(f"[CONTACT] Force spike: {delta_force_z:.2f} N.")
            print(f"[CONTACT] Gripper at Absolute Z = {gripper_world_z:.4f} m\n")
            return gripper_world_z # Return this value for the Auto-TCP math
            
        robot.rtde_c.speedL(speedl_command, acceleration=0.5, time=0.02)
        time.sleep(0.002)
        
def translate_until_contact_x(robot, speed_x, downward_force=5.0):
    """
    Drives laterally (X-axis) while actively maintaining a downward pinning force (Z-axis).
    Stops when the lateral force limit is hit.
    Returns the absolute World X and Y coordinates at the moment of impact.
    """
    robot.zero_ft_sensor()
    time.sleep(0.5)
    
    baseline_ft = robot.get_ft_sensor_baserot(world_frame=True)
    baseline_f_x = baseline_ft[0]
    
    print(f"\n[SEARCH] Translating X at {abs(speed_x)*100} cm/s with {downward_force} N downward pressure.")

    theta = np.deg2rad(45)
    R_world_base = np.array([
        [np.cos(theta),  0.0, np.sin(theta)],
        [0.0,            1.0, 0.0          ],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])
    
    # Active Force Control parameter
    Z_FORCE_GAIN = 0.002  # m/s per Newton of error
    
    # Throttle timer for terminal printing
    last_print_time = time.time()
    PRINT_RATE = 0.25 
    
    while True:
        current_ft = robot.get_ft_sensor_baserot(world_frame=True)
        f_x, f_y, f_z, t_x, t_y, t_z = current_ft
        
        delta_force_x = abs(f_x - baseline_f_x)
        
        # --- ACTIVE Z-FORCE CONTROL ---
        # Calculate the error between what we want (e.g., 5N) and what we feel
        f_z_error = downward_force - f_z
        
        # Calculate the required Z-velocity to fix the error.
        # If f_z is 0N, error is +5. Output is NEGATIVE (moves DOWN to push harder).
        # If f_z is 10N, error is -5. Output is POSITIVE (yields UP to relieve pressure).
        v_z_world = -(f_z_error * Z_FORCE_GAIN)
        
        # Cap the Z-velocity at 2 cm/s so it doesn't bounce violently or plunge off edges
        v_z_world = np.clip(v_z_world, -0.02, 0.02) 

        # --- COMBINE VELOCITIES ---
        V_world = np.array([speed_x, 0.0, v_z_world])
        V_base = R_world_base.T @ V_world 
        speedl_command = [V_base[0], V_base[1], V_base[2], 0.0, 0.0, 0.0]
        
        # --- TERMINAL DASHBOARD ---
        current_time = time.time()
        if (current_time - last_print_time) > PRINT_RATE:
            print(f"\r[SEARCH] Fx:{f_x:5.1f}N | Fz:{f_z:5.1f}N (Target:{downward_force}N) | Vz:{v_z_world*100:5.2f}cm/s", end="")
            last_print_time = current_time
            
        # --- COLLISION CHECK (LATERAL) ---
        if delta_force_x > FORCE_LATERAL_LIMIT:
            robot.rtde_c.speedStop() # INSTANT BRAKE
            print() # Clear the live dashboard line
            
            # Capture the exact pose of the TCP
            current_pose = robot.rtde_r.getActualTCPPose()
            p_base_h = np.array([current_pose[0], current_pose[1], current_pose[2], 1.0])
            
            T_world_base = np.eye(4)
            T_world_base[:3, :3] = R_world_base
            p_world_h = T_world_base @ p_base_h
            
            contact_x = p_world_h[0]
            contact_y = p_world_h[1]
            contact_z = p_world_h[2]
            
            print(f"[CONTACT] Lateral Force spike: {delta_force_x:.2f} N.")
            print(f"[CONTACT] Wall found at World XY = ({contact_x:.4f}, {contact_y:.4f}, {contact_z:.4f})\n")
            return contact_x, contact_y, contact_z
            
        # --- SAFETY BRAKE ---
        # Prevent runaway Z-forces if the robot hits an angled ramp or immovable wedge
        if abs(f_z) > 30.0:
            print(f"\n[DANGER] Z-Force runaway ({f_z:.1f} N). Halting!")
            robot.rtde_c.speedStop()
            return None, None
            
        robot.rtde_c.speedL(speedl_command, acceleration=0.8, time=0.02)
        time.sleep(0.002)
        
def move_compliant_world(robot, target_x_w, target_y_w, target_z_w, base_speed=0.015, downward_force=None):
    """
    Drives toward a World XYZ target. If downward_force is provided, it actively pins the Z-axis 
    while dragging X and Y to their targets kinematically.
    """
    print(f"[ACTION] Initiating compliant move to World XYZ: ({target_x_w:.4f}, {target_y_w:.4f}, {target_z_w:.4f})")
    
    theta = np.deg2rad(45)
    R_world_base = np.array([
        [np.cos(theta),  0.0, np.sin(theta)],
        [0.0,            1.0, 0.0          ],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])

    robot.zero_ft_sensor()
    time.sleep(0.5)
    
    TOLERANCE = 0.002        
    
    while True:
        current_pose = robot.rtde_r.getActualTCPPose()
        p_base = np.array([current_pose[0], current_pose[1], current_pose[2]])
        p_world = R_world_base @ p_base 
        
        dx_w = target_x_w - p_world[0]
        dy_w = target_y_w - p_world[1]
        dz_w = target_z_w - p_world[2]
        
        distance = np.sqrt(dx_w**2 + dy_w**2 + dz_w**2)
        dist_xy = np.sqrt(dx_w**2 + dy_w**2)
        
        # --- SUCCESS EXIT ---
        # If pinning, we only care that the XY sweep finished.
        if downward_force is not None and dist_xy <= TOLERANCE:
            robot.rtde_c.speedStop()
            print("[SUCCESS] XY Target reached compliantly with active Z-pinning.")
            return True
        elif downward_force is None and distance <= TOLERANCE:
            robot.rtde_c.speedStop()
            print("[SUCCESS] 3D Target reached compliantly.")
            return True
            
        v_nominal_w = np.array([
            (dx_w / distance) * base_speed,
            (dy_w / distance) * base_speed,
            (dz_w / distance) * base_speed
        ])
        
        current_ft_world = robot.get_ft_sensor_baserot(world_frame=True)
        f_x, f_y, f_z, t_x, t_y, t_z = current_ft_world
        
        # --- SAFETY EXIT ---
        if abs(f_x) > 20.0 or abs(f_y) > 20.0 or abs(f_z) > 40.0:
            print(f"\n[DANGER] Runaway force detected (X:{f_x:.1f}, Y:{f_y:.1f}, Z:{f_z:.1f}) N.")
            robot.rtde_c.speedStop()
            return False
            
        XY_COMPLIANCE_GAIN = 0.002   
        Z_COMPLIANCE_GAIN = 0.0005 
        Z_FORCE_GAIN = 0.002  
        
        v_yield_w = np.array([0.0, 0.0, 0.0])
        if abs(f_x) > 5.0: v_yield_w[0] = f_x * XY_COMPLIANCE_GAIN 
        if abs(f_y) > 5.0: v_yield_w[1] = f_y * XY_COMPLIANCE_GAIN
        
        # --- THE ACTIVE Z TOGGLE ---
        if downward_force is not None:
            f_z_error = downward_force - f_z
            v_z_active = -(f_z_error * Z_FORCE_GAIN)
            v_z_active = np.clip(v_z_active, -0.02, 0.02)
            # Completely override the nominal Z with our force-control Z
            v_nominal_w[2] = v_z_active
        else:
            # Standard passive yield if no force is requested
            if abs(f_z) > 5.0: v_yield_w[2] = f_z * Z_COMPLIANCE_GAIN
            
        v_final_world = v_nominal_w + v_yield_w
        v_final_base = R_world_base.T @ v_final_world
        
        speedl_command = [v_final_base[0], v_final_base[1], v_final_base[2], 0.0, 0.0, 0.0]
        robot.rtde_c.speedL(speedl_command, acceleration=0.8, time=0.02)
        time.sleep(0.002)
        
def execute_compliant_arch(robot, center_x_w, center_y_w, center_z_w, base_speed=0.015, downward_force=None):
    """
    Sweeps the TCP along a perfect mathematical arc anchored at the socket hole, 
    stopping when the TCP is perfectly vertically aligned over the center.
    """
    print(f"[ACTION] Initiating Arch Pivot over fulcrum XYZ: ({center_x_w:.4f}, {center_y_w:.4f}, {center_z_w:.4f})")
    
    theta = np.deg2rad(45)
    R_world_base = np.array([
        [np.cos(theta),  0.0, np.sin(theta)],
        [0.0,            1.0, 0.0          ],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])

    robot.zero_ft_sensor()
    time.sleep(0.5)
    
    TOLERANCE = 0.002        
    
    while True:
        current_pose = robot.rtde_r.getActualTCPPose()
        p_base = np.array([current_pose[0], current_pose[1], current_pose[2]])
        p_world = R_world_base @ p_base 
        
        # Calculate vector from the socket hole (fulcrum) to the TCP
        dx_c = p_world[0] - center_x_w
        dz_c = p_world[2] - center_z_w
        
        # --- SUCCESS EXIT ---
        # Stop when X distance to center is less than 2mm (rod is straight up)
        if abs(dx_c) <= TOLERANCE:
            robot.rtde_c.speedStop()
            print("[SUCCESS] Arch completed. Rod is vertical.")
            return True
            
        # --- CALCULATE TANGENT VELOCITY (THE ARCH) ---
        # The tangent vector dictates a perfect circular path
        if dx_c > 0: # TCP is on the right, needs to sweep left and UP
            tan_x = -dz_c
            tan_z = dx_c
        else:        # TCP is on the left, needs to sweep right and UP
            tan_x = dz_c
            tan_z = -dx_c
            
        tan_mag = math.sqrt(tan_x**2 + tan_z**2)
        
        # This scales the velocity to perfectly trace the arc
        v_nom_x = (tan_x / tan_mag) * base_speed
        v_nom_y = (center_y_w - p_world[1]) * 1.5 # Gently autocorrect any Y-axis drifting
        v_nom_z = (tan_z / tan_mag) * base_speed
        
        v_nominal_w = np.array([v_nom_x, v_nom_y, v_nom_z])
        
        current_ft_world = robot.get_ft_sensor_baserot(world_frame=True)
        f_x, f_y, f_z, t_x, t_y, t_z = current_ft_world
        
        # --- SAFETY EXIT ---
        if abs(f_x) > 20.0 or abs(f_y) > 20.0 or abs(f_z) > 40.0:
            print(f"\n[DANGER] Runaway force detected.")
            robot.rtde_c.speedStop()
            return False
            
        # --- ACTIVE Z-PINNING ALONG THE ARCH ---
        v_yield_w = np.array([0.0, 0.0, 0.0])
        XY_COMPLIANCE_GAIN = 0.002   
        Z_COMPLIANCE_GAIN = 0.0005 
        Z_FORCE_GAIN = 0.002
        
        if abs(f_x) > 5.0: v_yield_w[0] = f_x * XY_COMPLIANCE_GAIN 
        if abs(f_y) > 5.0: v_yield_w[1] = f_y * XY_COMPLIANCE_GAIN
        
        if downward_force is not None:
            f_z_error = downward_force - f_z
            v_z_active = -(f_z_error * Z_FORCE_GAIN)
            v_z_active = np.clip(v_z_active, -0.015, 0.015)
            # Add the pinning pressure TO the mathematical arc (don't overwrite it)
            v_nominal_w[2] += v_z_active
        else:
            if abs(f_z) > 5.0: v_yield_w[2] = f_z * Z_COMPLIANCE_GAIN
            
        v_final_world = v_nominal_w + v_yield_w
        v_final_base = R_world_base.T @ v_final_world
        
        speedl_command = [v_final_base[0], v_final_base[1], v_final_base[2], 0.0, 0.0, 0.0]
        robot.rtde_c.speedL(speedl_command, acceleration=0.8, time=0.02)
        time.sleep(0.002)


def main():
    print("[INIT] Connecting to UR5e Controller...")
    robot = UR5eSafeController(ip=ip)
    
    print("[INIT] Connecting to RG2 Gripper...")
    rg_gripper = RG2(ip, rg_id)
    
    try:
        # PRE-FLIGHT & ROD LOADING (TIMED)
        #print("\n[ACTION] Opening gripper for rod loading. You have 5 seconds...")
        #rg_gripper.rg_grip(GRIPPER_OPEN, GRIPPER_FORCE)

        
        print(f"[ACTION] Gripping rod at {GRIPPER_FORCE} N...")
        rg_gripper.rg_grip(GRIPPER_CLOSED, GRIPPER_FORCE)
        time.sleep(2.0)
        
        init_pose = robot.rtde_r.getActualTCPPose()
        print(f"[STATUS] Initial Raw Base TCP: X={init_pose[0]:.4f}, Y={init_pose[1]:.4f}, Z={init_pose[2]:.4f}")

        #---------------------------------#
        # PHASE 1: AUTO-CALIBRATION PLUNGE
        print(f"\n ---------- PHASE 1 ----------")
        print(f"\n[PHASE 1] Moving to Safe Approach Height ({SAFE_APPROACH_Z} m)...")
        robot.move_to_xyz_safe(START_X, START_Y, SAFE_APPROACH_Z, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
        
        # Plunge and get the height of the gripper fingers
        contact_gripper_z_table = plunge_until_contact(robot)
        
        # --- THE AUTO-TCP MATH ---
        rod_length = contact_gripper_z_table - TABLE_TOP_Z
        print(f"Contact with Table Surface at: {contact_gripper_z_table:.2f}")
        print(f"[INFO] Calculated Rod Length: {rod_length * 100:.2f} cm")
        
        #---------------------------------#
        # Phase 2: Adjust to 45 degrees
        print(f"\n ---------- PHASE 2 ----------")
        robot.move_to_xyz_safe(START_X, START_Y, SOCKET_SURFACE_Z + rod_length + SAFETY_OFFSET, 			# Move to safe height
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
        robot.move_to_xyz_safe(Socket_left_X, Socket_left_Y, SOCKET_SURFACE_Z + rod_length + SAFETY_OFFSET, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
        robot.move_to_xyz_safe(Socket_left_X, Socket_left_Y, SOCKET_SURFACE_Z + rod_length - SAFETY_OFFSET, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
        
        current_pose_2 = robot.rtde_r.getActualTCPPose()
        print(f"Current Position: Socket Far Contact {current_pose_2[2]:.4f}")
        
        time.sleep(1.0)                       
        print(f"\n[PHASE 2] Loosening Grip...")
        rg_gripper.rg_grip(GRIPPER_COMPLIANCE, GRIPPER_COMP_FORCE)
        time.sleep(2.0)      
        
        # --- THE PIVOT GEOMETRY ---
        PIVOT_ANGLE_DEG = 45.0
        theta = math.radians(PIVOT_ANGLE_DEG)
        SOCKET_DEPTH = 0 # 2 cm
        # The length of the rod sticking out above the socket lip
        effective_length = rod_length - SOCKET_DEPTH
        # SOH CAH TOA: Calculate the exact X and Z distances from the socket lip
        delta_x = effective_length * math.cos(theta)
        delta_z = effective_length * math.sin(theta)

        # Calculate your final target depth based on your custom math
        target_x_world = Socket_left_X + delta_x 
        target_y_world = Socket_left_Y 
        target_z_world = SOCKET_SURFACE_Z - SAFETY_OFFSET + delta_z
        print(f"Delta calculated X: {delta_x:.4f} and Z: {delta_z:.4f}")
        
        # Call the Virtual Spring (Slowed down to 3 cm/s for safety during testing)
        move_compliant_world(robot, target_x_world, target_y_world, target_z_world, base_speed=0.02)
        time.sleep(1.0)
        
        #---------------------------------#      
        # Phase 3: Move back to right of socket
        print(f"\n ---------- PHASE 3 ----------")    
        print(f"\n[PHASE 3] Moving to starting side of socket...")    
        robot.move_to_xyz_safe(target_x_world, target_y_world, target_z_world + rod_length, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)             
        robot.move_to_xyz_safe(Socket_start_X + delta_x, Socket_start_Y, target_z_world + 0.06, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
                      
        contact_gripper_socket_top45 = plunge_until_contact(robot)
        
        # Search for Socket edge 
        found_x, found_y, found_z = translate_until_contact_x(robot, LATERAL_SPEED, downward_force=5.0)
        print(f"\n[PHASE 3] Initiating blind lateral search in -X direction...")   
        
        robot.move_to_xyz_safe(found_x + (hole_diameter / 2), found_y, found_z, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
        
        current_pose_insertion = robot.rtde_r.getActualTCPPose()
        print(f"Current Pose: {current_pose_insertion}")
        rg_gripper.rg_grip(GRIPPER_COMPLIANCE, GRIPPER_COMP_FORCE)
        time.sleep(2.0)  
        
        # 45 degrees stands the rod perfectly straight along the Z-axis
        PIVOT_ANGLE_DEG = 45.0
        theta = math.radians(PIVOT_ANGLE_DEG)
        SOCKET_DEPTH = 0.005 # 5 mm deep inside the lip
        effective_length = rod_length - SOCKET_DEPTH
        delta_x = effective_length * math.cos(theta) # Will be 0.0
        delta_z = effective_length * math.sin(theta) # Will be effective_length

        hole_center_x = -0.0983
        hole_center_y = found_y
        hole_center_z = SOCKET_SURFACE_Z
        print(f"[MATH] Straightening Pivot - X offset: {delta_x:.4f}, Z offset: {delta_z:.4f}")
        
        print("\n[ACTION] Loosening Gripper...")
        rg_gripper.rg_grip(21.5, GRIPPER_FORCE) ##### Next Steps: Maybe add a contact in z before the arc
        time.sleep(0.5)
        
        # Arc to Insertion point
        execute_compliant_arch(robot, hole_center_x, hole_center_y, hole_center_z, base_speed=0.02, downward_force=8.0)
        
        current_pose_insertion = robot.rtde_r.getActualTCPPose()
        print(f"Current Pose: {current_pose_insertion}")
        time.sleep(0.5)
        
        # Final Plunge straight down into the socket
        print(f"\n[PHASE 3] Executing final compliant insertion...")
        #move_compliant_world(robot, target_x_world, target_y_world, insertion_target_z, base_speed=0.01)
        
        print("\n[SUCCESS] Autonomous tactile sequence complete.")
        
        # Clean Up
        #print("\n[ACTION] Releasing Rod...")
        #rg_gripper.rg_grip(GRIPPER_OPEN, GRIPPER_FORCE)
        #print("[SUCCESS] Autonomous tactile sequence complete.")

    except Exception as e:
        print(f"[ERROR] Sequence aborted: {e}")
        robot.rtde_c.stopJ(2.0)
    finally:
        robot.cleanup()

if __name__ == "__main__":
    main()
