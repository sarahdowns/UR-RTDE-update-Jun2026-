import math
import numpy as np
import ur_dual_controller as urcx
from trajectory import Trajectory

# --- Configuration ---
# Your confirmed working IP addresses
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# --- Speed Control ---
# The time in seconds for each *segment* of the oscillation (e.g., from +1 to -1).
# Larger value = SLOWER movement.
MOVEMENT_SPEED_SECONDS = 2.0

# --- Oscillation Parameters ---
OSCILLATION_DELTA = 1.0  # The amount in radians to move the joint
REPETITIONS = 2          # The number of times to oscillate back and forth

print("=" * 60)
print("Starting Smoothed Synchronized Movement Test")
print("=" * 60)
print("WARNING: This script will move the robots. Press Ctrl+C to stop.")
print("Ensure the area around the robots is clear.")
print("=" * 60)

try:
    # 1. Initialize the dual controller with the correct parameters
    print("\n[Step 1] Initializing URDualController...")
    rbtx = urcx.URDualController(
        master_robot_ip=MASTER_ROBOT_IP,
        slave_robot_ip=SLAVE_ROBOT_IP,
        control_pc_ip=CONTROL_PC_IP
    )
    print("✓ Controller initialized successfully.")

    # 2. Get the current joint positions of both arms
    # Access the arms using the correct private attributes (_lft_arm, _rgt_arm)
    # and the correct method (getj)
    print("\n[Step 2] Reading initial joint positions...")
    current_lft_jnt_values = np.array(rbtx._lft_arm.getj())
    current_rgt_jnt_values = np.array(rbtx._rgt_arm.getj())
    print("✓ Initial positions read.")
    print(f"  Master Arm: {np.round(current_lft_jnt_values, 2)}")
    print(f"  Slave Arm:  {np.round(current_rgt_jnt_values, 2)}")

    # 3. Generate a smooth, looping trajectory
    print("\n[Step 3] Generating a smooth, looping trajectory...")
    
    # First, define the key waypoints for the oscillation
    lft_waypoint_plus = current_lft_jnt_values.copy()
    lft_waypoint_plus[3] += OSCILLATION_DELTA
    lft_waypoint_minus = current_lft_jnt_values.copy()
    lft_waypoint_minus[3] -= OSCILLATION_DELTA

    rgt_waypoint_plus = current_rgt_jnt_values.copy()
    rgt_waypoint_plus[3] += OSCILLATION_DELTA
    rgt_waypoint_minus = current_rgt_jnt_values.copy()
    rgt_waypoint_minus[3] -= OSCILLATION_DELTA
    
    # Build the list of key waypoints for each arm
    key_waypoints_lft = [current_lft_jnt_values]
    key_waypoints_rgt = [current_rgt_jnt_values]
    for _ in range(REPETITIONS):
        key_waypoints_lft.append(lft_waypoint_plus)
        key_waypoints_rgt.append(rgt_waypoint_plus)
        key_waypoints_lft.append(lft_waypoint_minus)
        key_waypoints_rgt.append(rgt_waypoint_minus)
    key_waypoints_lft.append(current_lft_jnt_values)
    key_waypoints_rgt.append(current_rgt_jnt_values)

    # Now, use the Trajectory class to generate smooth paths between these waypoints
    traj_generator = Trajectory(method="quintic")
    
    # Generate the smooth path for the left arm
    path_lft, _ = traj_generator.piecewise_interpolation(
        path=key_waypoints_lft,
        control_frequency=0.008, # UR robot's control frequency (1/125Hz)
        interval_time=MOVEMENT_SPEED_SECONDS
    )
    
    # Generate the smooth path for the right arm
    path_rgt, _ = traj_generator.piecewise_interpolation(
        path=key_waypoints_rgt,
        control_frequency=0.008, # UR robot's control frequency (1/125Hz)
        interval_time=MOVEMENT_SPEED_SECONDS
    )

    print(f"✓ Smooth trajectory generated with {len(path_lft)} waypoints.")

    # Combine the left and right arm paths for the controller
    combined_path = [lft + rgt for lft, rgt in zip(path_lft, path_rgt)]

    # 4. Execute the synchronized movement
    print("\n[Step 4] Sending trajectory to robots for synchronized movement...")
    # NOTE: The interval_time here is now for the *interpolated* points. 
    # It should be very small. The overall speed is controlled by the 
    # MOVEMENT_SPEED_SECONDS used during trajectory generation.
    print("!!! ROBOTS WILL NOW MOVE SMOOTHLY !!!")
    rbtx.move_jntspace_path(combined_path, control_frequency=0.008, interval_time=0.008)
    print("\n✓ Movement command sent and executed.")

except KeyboardInterrupt:
    print("\n\nScript interrupted by user. Stopping robots.")
except Exception as e:
    print(f"\n✗ An error occurred: {e}")
finally:
    # 5. Cleanly close the connections
    if 'rbtx' in locals():
        rbtx._lft_arm.close()
        rbtx._rgt_arm.close()
        print("\n[Step 5] Robot connections closed.")

print("\nTest finished.")