# File name: test_move_setpoint.py

from move_xyz_safe import UR5eSafeController
from gripper_RG2 import RG2
import numpy as np

ip = "192.168.5.5"
rg_id = 0
rg_gripper = RG2(ip, rg_id)
arm = UR5eSafeController(ip=ip)

initial_home_q = arm.rtde_r.getActualQ()
        
def main():
    robot = UR5eSafeController(ip=ip)
    try:    	
        robot.zero_ft_sensor()
        # Hardcoded target
        x, y, z = .3, -.5, .1
        	# negative y is forward
        	# positive z in up
        pitch_deg = 180  # straight down
        rg_gripper.rg_grip(60, 25.0)
        robot.move_to_xyz_safe(x, y, z + 0.05, pitch_deg=pitch_deg, speed=0.15, acceleration=0.2)
       # robot.move_to_xyz_safe(x, y, z, pitch_deg=pitch_deg, speed=0.2, acceleration=0.5)
        
       
   	# 3. Capture baseline force before the final descentn
        baseline = robot.get_ft_sensor_data()
        print(f"Baseline Force (Z): {baseline[2]:.2f} N")

        # 4. Check force
        if robot.move_to_xyz_safe(x, y, z, pitch_deg=pitch_deg):
            final_ft = robot.get_ft_sensor_data()
            contact_force_z = final_ft[2] - baseline[2] # Subtract baseline to isolate contact
            force_vector = final_ft[:3]
            total_force = np.linalg.norm(force_vector)
            
            print(f"Resulting Contact Force (X): {final_ft[0]:.2f} N")
            print(f"Resulting Contact Force (Y): {final_ft[1]:.2f} N")
            print(f"Resulting Contact Force (Z): {final_ft[2]:.2f} N")
            print(f"Resulting Total Contact Force: {total_force:.2f} N")
            print('-'*30)
            print(f"Resulting Torque around X: {final_ft[3]:.2f} Nm")
            print(f"Resulting Torque around Y: {final_ft[4]:.2f} Nm")
            print(f"Resulting Torque around X: {final_ft[5]:.2f} Nm")
            
            if abs(contact_force_z) > 30.0:
                print("Force exceeds 80% of 50N limit")
        pass
    except Exception as e:
        print(f"Caught error: {e}")
        
    finally:
        robot.reset_protective_stop()
        robot.cleanup()

if __name__ == "__main__":
    main()
