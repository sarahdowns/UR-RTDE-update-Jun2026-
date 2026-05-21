# Move down in z until contact

from move_xyz_safe import UR5eSafeController
import numpy as np
from spatialmath import SE3

def main():
    # 1. Initialize your existing class
    # The class applies: self.robot.base = SE3.Ry(np.deg2rad(45))
    ctrl = UR5eSafeController(ip="192.168.5.5")
    
    # 2. Define Velocity in World Frame (-3cm/s straight down to floor)
    v_world = np.array([0, 0, -0.03, 0, 0, 0]) 
    
    # 3. Transform Velocity to Base Frame
    # To move in World -Z, the robot must move diagonally in its Base Frame
    # We rotate the linear velocity components (index 0:3) by -45 degrees
    R_world_to_base = ctrl.robot.base.R.T 
    v_base_linear = R_world_to_base @ v_world[:3]
    speed_vector = list(v_base_linear) + [0, 0, 0]
    
    try:
        print("-" * 30)
        print(f"Moving in World -Z at 3cm/s...")
        
        ctrl.rtde_c.moveUntilContact(speed_vector)
        q_contact = ctrl.rtde_r.getActualQ()
        
        # Calculate World Frame Pose (Includes 45-deg tilt)
        T_world = ctrl.robot.fkine(q_contact)
        
        # Calculate Base Frame Pose (Ignores tilt)
        # Temporarily clear base to get raw coordinates relative to mounting plate
        original_base = ctrl.robot.base
        ctrl.robot.base = SE3()
        T_base = ctrl.robot.fkine(q_contact)
        ctrl.robot.base = original_base # Restore tilt
        
        # 6. Output Comparison
        print("-" * 30)
        print("CONTACT DETECTED")
        print("-" * 30)
        print("WORLD FRAME (TCP relative to floor):")
        print(f"X: {T_world.t[0]:.4f}m, Y: {T_world.t[1]:.4f}m, Z: {T_world.t[2]:.4f}m")
        
        print("\nBASE FRAME (TCP relative to mounting plate):")
        print(f"X: {T_base.t[0]:.4f}m, Y: {T_base.t[1]:.4f}m, Z: {T_base.t[2]:.4f}m")
        print("-" * 30)

        # 7. Safety retreat (World Frame +2cm)
        retreat_world = list(T_world.t)
        retreat_world[2] += 0.02
        # Use your safe move method to return to a safe height
        ctrl.move_to_xyz_safe(retreat_world[0], retreat_world[1], retreat_world[2], visualize=False)

    except Exception as e:
        print(f"[ERROR] Contact command failed: {e}")
    finally:
        ctrl.cleanup()
        print("RTDE interfaces closed.")

if __name__ == "__main__":
    main()
    
# World vs Base Frame Check
"""
from move_xyz_safe import UR5eSafeController
import numpy as np
from spatialmath import SE3

def run_frame_diagnostic():
    # 1. Initialize the controller with the 45-deg tilt
    # self.robot.base = SE3.Ry(np.deg2rad(45)) is applied here
    ctrl = UR5eSafeController(ip="192.168.5.5")
    
    try:
        # 2. Get the current joint angles (q)
        current_q = ctrl.rtde_r.getActualQ()
        
        # 3. Calculate Forward Kinematics (FK)
        # World Frame (accounts for the 45-deg base tilt)
        T_world = ctrl.robot.fkine(current_q)
        
        # Base Frame (ignores the tilt, relative only to the base plate)
        # We do this by temporarily stripping the base transform
        ctrl.robot.base = SE3() 
        T_base = ctrl.robot.fkine(current_q)
        
        # 4. Print Comparison
        print("-" * 30)
        print("COORDINATE FRAME DIAGNOSTIC")
        print("-" * 30)
        print(f"WORLD FRAME (Tilted 45°):")
        print(f"X: {T_world.t[0]:.4f}m, Y: {T_world.t[1]:.4f}m, Z: {T_world.t[2]:.4f}m")
        print("\nBASE FRAME (Mounting Plate):")
        print(f"X: {T_base.t[0]:.4f}m, Y: {T_base.t[1]:.4f}m, Z: {T_base.t[2]:.4f}m")
        print("-" * 30)
        
        # 5. Verification Math
        # At a 45-degree tilt, the relationship should be:
        # World_Z = (Base_Z * cos(45)) - (Base_X * sin(45))
        expected_z = (T_base.t[2] * np.cos(np.pi/4)) - (T_base.t[0] * np.sin(np.pi/4))
        print(f"Tilt Consistency Check: {abs(expected_z - T_world.t[2]):.6f} deviation")

    finally:
        ctrl.cleanup()

if __name__ == "__main__":
    run_frame_diagnostic()
    
"""
