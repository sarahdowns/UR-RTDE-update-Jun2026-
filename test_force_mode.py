import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from move_xyz_safe import UR5eSafeController

ip = "192.168.5.4" # Updated per your snippet
WINDOW_SIZE = 100 
task_frame = [0, 0, 0, 0, 0, 0] 
 
# [X, Y, Z, Rx, Ry, Rz] -> Compliant where set to "1"
selection_vector = [1, 1, 1, 0, 0, 0] 

wrench = [0, 0, 0, 0, 0, 0] 
force_type = 2 
limits = [2.0, 2.0, 1.5, 1.0, 1.0, 1.0]

def main():
    print(f"Initializing UR5e Controller at {ip}...")
    # Initialize your custom controller
    robot = UR5eSafeController(ip)

    # Initialize data buffers
    fx_data = deque(maxlen=WINDOW_SIZE)
    fy_data = deque(maxlen=WINDOW_SIZE)
    fz_data = deque(maxlen=WINDOW_SIZE)

    # Setup Matplotlib
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    line_x, = ax.plot([], [], label='World Force X', color='r', linewidth=1.5)
    line_y, = ax.plot([], [], label='World Force Y', color='g', linewidth=1.5)
    line_z, = ax.plot([], [], label='World Force Z', color='b', linewidth=1.5)
    
    ax.set_ylim(-60, 60)
    ax.set_ylabel("Force (Newtons)")
    ax.set_xlabel("Samples")
    ax.set_title(f"Live World-Frame Force Feedback (-45° Base Offset Applied)")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    try:
        # Zero the sensor in the current orientation to remove gripper weight
        robot.zero_ft_sensor()
        time.sleep(1.0) # Give it a second to settle
        
        print("\n[ACTIVE] Force Mode Engaged (X and Z Compliant).")
        print("The robot will now feel 'soft' if pushed in X or Z.")
        print("press Ctrl+C twice to end")
        
        while True:
            # 1. Maintain Force Mode (Using the rtde_c interface within the controller)
            robot.rtde_c.forceMode(task_frame, selection_vector, wrench, force_type, limits)

            # 2. Get Rotated Data from your custom method
            # world_frame=True uses the Ry(-45) matrix from your move_xyz_safe setup
            ft_data = robot.get_ft_sensor_baserot(world_frame=True)
            fx, fy, fz = ft_data[0], ft_data[1], ft_data[2]
            
            if abs(fx) < 1.0:
                fx = 0.0
            if abs(fy) < 1.0:
                fy = 0.0
            if abs(fz) < 1.0:
                fz = 0.0
                
            # --- IMPACT SAFETY LIMIT ---
            MAX_FORCE = 80.0   # Newtons
            
            if abs(fx) > MAX_FORCE or abs(fy) > MAX_FORCE or abs(fz) > MAX_FORCE:
                print("[SAFETY] Excessive force detected!")
                break

            # 3. Update Buffers
            fx_data.append(fx)
            fy_data.append(fy)
            fz_data.append(fz)

            # 4. Update Plot Lines
            x_range = range(len(fx_data))
            line_x.set_data(x_range, list(fx_data))
            line_y.set_data(x_range, list(fy_data))
            line_z.set_data(x_range, list(fz_data))

            ax.set_xlim(max(0, len(fx_data)-WINDOW_SIZE), len(fx_data))
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            time.sleep(0.01) # ~100Hz

    except KeyboardInterrupt:
        print("\nManual Stop Detected.")
    finally:
        # Use the controller's internal interfaces to stop safely
        robot.rtde_c.forceModeStop()
        robot.cleanup()
        print("[CLEANUP] Force mode disabled. Robot Locked.")
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
