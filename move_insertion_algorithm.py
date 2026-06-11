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

# Position 1 (Initial Starting Point)
START_X = 0.15
START_Y = -0.6

Socket_start_X = -0.07
Socket_start_Y = -0.6

Socket_left_X = -0.18
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
            
            print(f"[CONTACT] Lateral Force spike: {delta_force_x:.2f} N.")
            print(f"[CONTACT] Wall found at World XY = ({contact_x:.4f}, {contact_y:.4f})\n")
            return contact_x, contact_y 
            
        # --- SAFETY BRAKE ---
        # Prevent runaway Z-forces if the robot hits an angled ramp or immovable wedge
        if abs(f_z) > 30.0:
            print(f"\n[DANGER] Z-Force runaway ({f_z:.1f} N). Halting!")
            robot.rtde_c.speedStop()
            return None, None
            
        robot.rtde_c.speedL(speedl_command, acceleration=0.8, time=0.02)
        time.sleep(0.002)
        
def move_compliant_world(robot, target_x_w, target_y_w, target_z_w, base_speed=0.015):
    """
    Drives toward a World XYZ target using velocity control. If lateral forces rise, the robot yields (moves away from the force) 
    to slide smoothly along obstacles until it reaches the target.
    """
    print(f"[ACTION] Initiating compliant move to World XYZ: ({target_x_w:.4f}, {target_y_w:.4f}, {target_z_w:.4f})")
    
    # 1. Define the 45-degree rotation matrix to convert World to Base
    theta = np.deg2rad(45)
    R_world_base = np.array([
        [np.cos(theta),  0.0, np.sin(theta)],
        [0.0,            1.0, 0.0          ],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])

    robot.zero_ft_sensor()
    time.sleep(0.5)
    
    # Tuning parameters for the "Virtual Spring"
    COMPLIANCE_GAIN = 0.002  # How much velocity to subtract per Newton of force
    TOLERANCE = 0.002        # Stop when within 2mm of the target
    
    while True:
        current_pose = robot.rtde_r.getActualTCPPose()
        p_base = np.array([current_pose[0], current_pose[1], current_pose[2]])
        p_world = R_world_base @ p_base # Convert current position to World Frame
        
        # --- DISTANCE TO TARGET ---
        dx_w = target_x_w - p_world[0]
        dy_w = target_y_w - p_world[1]
        dz_w = target_z_w - p_world[2]
        
        distance = np.sqrt(dx_w**2 + dy_w**2 + dz_w**2)
        
        # Did we reach the target?
        if distance <= TOLERANCE:
            robot.rtde_c.speedStop()
            print("[SUCCESS] Target reached compliantly.")
            break
            
        # --- CALCULATE NOMINAL VELOCITY ---
        # Normalize the vector and multiply by our base speed
        v_nominal_w = np.array([
            (dx_w / distance) * base_speed,
            (dy_w / distance) * base_speed,
            (dz_w / distance) * base_speed
        ])

        current_ft_world = robot.get_ft_sensor_baserot(world_frame=True)
        f_x = current_ft_world[0]
        f_y = current_ft_world[1]
        f_z = current_ft_world[2] 
        
        # Software Safety Brake! 
        if abs(f_x) > 20.0 or abs(f_y) > 20.0 or abs(f_z) > 40.0:
            print(f"\n[DANGER] Runaway force detected (X:{f_x:.1f}, Y:{f_y:.1f}, Z:{f_z:.1f}) N.")
            print("[DANGER] Software Safety Brake Engaged! Halting to prevent Protective Stop.")
            robot.rtde_c.speedStop()
            break
            
        #print(f"\n[INFO] Force detected (X:{f_x:.1f}, Y:{f_y:.1f}, Z:{f_z:.1f}) N.")	# print through motion
        # Tuning parameters for the "Virtual Spring"
        XY_COMPLIANCE_GAIN = 0.002   # Soft: Yields easily to walls
        Z_COMPLIANCE_GAIN = 0.0005   # Stiff: Requires 4x the force to yield!
        
        v_yield_w = np.array([0.0, 0.0, 0.0])
        
        if abs(f_x) > 5.0:
            v_yield_w[0] = f_x * XY_COMPLIANCE_GAIN 
        if abs(f_y) > 5.0:
            v_yield_w[1] = f_y * XY_COMPLIANCE_GAIN
        if abs(f_z) > 5.0:                           
            v_yield_w[2] = f_z * Z_COMPLIANCE_GAIN
            
        # --- 5. COMBINE AND EXECUTE ---
        # Simply ADD the calculated Yield Velocity directly to the Nominal Velocity
        v_final_world = v_nominal_w + v_yield_w
        
        # Convert the final world velocity back to the robot's raw base frame
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
        
        # 5-second wait limit instead of user input
        #time.sleep(3.0)
        
        print(f"[ACTION] Gripping rod at {GRIPPER_FORCE} N...")
        rg_gripper.rg_grip(GRIPPER_CLOSED, GRIPPER_FORCE)
        time.sleep(3.0)
        
        init_pose = robot.rtde_r.getActualTCPPose()
        print(f"[STATUS] Initial Raw Base TCP: X={init_pose[0]:.4f}, Y={init_pose[1]:.4f}, Z={init_pose[2]:.4f}")

        #---------------------------------#
        # PHASE 1: AUTO-CALIBRATION PLUNGE
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
        print(f"Calculated X: {delta_x:.4f} and Z: {delta_z:.4f}")
        
        # Call the Virtual Spring (Slowed down to 3 cm/s for safety during testing)
        move_compliant_world(robot, target_x_world, target_y_world, target_z_world, base_speed=0.02)
        
        #---------------------------------#
        # Phase 3: Move back to right of socket
        robot.move_to_xyz_safe(Socket_start_X, Socket_start_Y, SOCKET_SURFACE_Z + rod_length, 
                               visualize=False, roll_deg=ROLL_DEG, pitch_deg=PITCH_DEG, yaw_deg=YAW_DEG,
                               ask_user=False, async_move=False)
        
        # Search for Socket edge 
        found_x, found_y = translate_until_contact_x(robot, LATERAL_SPEED, downward_force=3.0)
        print(f"\n[PHASE 3] Initiating blind lateral search in -X direction...")                 
        
        #target_x_world = found_x - delta_x
        #target_y_world = found_y 
        #target_z_world = SOCKET_SURFACE_Z + delta_z
        #print(f"Calculated X: {delta_x:.4f} and Z: {delta_z:.4f}")
        
        # Call the Virtual Spring
        #move_compliant_world(robot, target_x_world, target_y_world, target_z_world, base_speed=0.03)
        
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
