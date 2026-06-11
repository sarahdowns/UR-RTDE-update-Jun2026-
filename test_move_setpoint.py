# File name: test_move_setpoint.pyy
# Target movement. Set elbow up or down preference

from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2
import numpy as np
import matplotlib.pyplot as plt

ip = "192.168.5.5"
rg_id = 0

# Set to "up", "down", or None (for shortest path)
ELBOW_PREFERENCE = "down" 

def main():
    print("[INIT] Connecting to UR5e Safe Controller...")
    robot = UR5eSafeController(ip=ip)
    
    print("[INIT] Connecting to OnRobot RG2 Gripper...")
    rg_gripper = RG2(ip, rg_id)
    
    try:        
        robot.zero_ft_sensor()
        
        # Hardcoded targets
        x, y, z = 0.2, -0.6, 0.25
        pitch_deg = 0
        roll_deg = -90
        yaw_deg = 90
        
        # 1. Grab current joints to manipulate the seed
        seed_q = robot.rtde_r.getActualQ()
        
        # The UR5e elbow joint is index 2. 
        # Positive usually forces Elbow Down, Negative forces Elbow Up.
        if ELBOW_PREFERENCE == "down":
            seed_q[2] = abs(seed_q[2]) 
            print("[INFO] IK Solver Seeded for ELBOW Down")
        elif ELBOW_PREFERENCE == "up":
            seed_q[2] = -abs(seed_q[2])
            print("[INFO] IK Solver Seeded for ELBOW Up")
        else:
            seed_q = None # Let the robot take the shortest natural path
            
        # 2. Capture baseline force before the final descent
        baseline = robot.get_ft_sensor_data()
        print(f"Baseline Force (Z): {baseline[2]:.2f} N")

        # 3. Execute move, passing in our customized seed
        if robot.move_to_xyz_safe(x, y, z, roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg, custom_seed=seed_q):
            
            final_ft = robot.get_ft_sensor_data()
            contact_force_z = final_ft[2] - baseline[2] 
            force_vector = final_ft[:3]
            total_force = np.linalg.norm(force_vector)
            
            print(f"Resulting Contact Force (X): {final_ft[0]:.2f} N")
            print(f"Resulting Contact Force (Y): {final_ft[1]:.2f} N")
            print(f"Resulting Contact Force (Z): {final_ft[2]:.2f} N")
            print(f"Resulting Total Contact Force: {total_force:.2f} N")
            print('-'*30)
            print(f"Resulting Torque around X: {final_ft[3]:.2f} Nm")
            print(f"Resulting Torque around Y: {final_ft[4]:.2f} Nm")
            print(f"Resulting Torque around Z: {final_ft[5]:.2f} Nm")
            
            if abs(contact_force_z) > 30.0:
                print("Force exceeds 30N limit")
        pass
    except Exception as e:
        print(f"Caught error: {e}")
        
    finally:
        robot.reset_protective_stop()
        robot.cleanup()

if __name__ == "__main__":
    main()
