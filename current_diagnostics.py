import numpy as np
from spatialmath import SE3
from move_xyz_safe import UR5eSafeController

# System Configuration
IP_ADDRESS = "192.168.5.5"

TILT = 45

def print_telemetry():
    print("\n[INIT] Connecting to UR5e Controller...")
    
    try:
        # Initialize your custom safe controller to access the RTDE interface
        robot = UR5eSafeController(ip=IP_ADDRESS)
        rtde_r = robot.rtde_r
        rtde_c = robot.rtde_c  # Needed for the TCP Offset command!
        
        # --- 1. JOINT DATA ---
        actual_q = rtde_r.getActualQ()
        actual_qd = rtde_r.getActualQd()
        
        print("\n" + "="*50)
        print(" 🦾 JOINT TELEMETRY (DEGREES)")
        print("="*50)
        labels = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
        for i in range(6):
            print(f"{labels[i]:<10}: Pos {np.rad2deg(actual_q[i]):>8.2f}° | Vel {np.rad2deg(actual_qd[i]):>6.2f}°/s")

        # --- 2. SPATIAL DATA (RAW BASE FRAME) ---
        tcp_base = rtde_r.getActualTCPPose()
        
        print("\n" + "="*50)
        print(" 📐 TCP SPATIAL POSE (RAW BASE FRAME)")
        print("="*50)
        print(f"X : {tcp_base[0]:>8.4f} m  |  Rx: {tcp_base[3]:>8.4f} rad")
        print(f"Y : {tcp_base[1]:>8.4f} m  |  Ry: {tcp_base[4]:>8.4f} rad")
        print(f"Z : {tcp_base[2]:>8.4f} m  |  Rz: {tcp_base[5]:>8.4f} rad")
        
        # --- 3. SPATIAL DATA (TILTED WORLD FRAME) ---
        # Apply your 45-degree Ry tilt to see where the robot thinks it is relative to the table
        T_world_base = SE3.Ry(np.deg2rad(TILT))
        tcp_base_h = np.array([tcp_base[0], tcp_base[1], tcp_base[2], 1.0])
        tcp_world = T_world_base.A @ tcp_base_h
        
        print("\n" + "="*50)
        print(" 🌍 TCP SPATIAL POSE (45° TILTED WORLD FRAME)")
        print("="*50)
        print(f"World X : {tcp_world[0]:>8.4f} m")
        print(f"World Y : {tcp_world[1]:>8.4f} m")
        print(f"World Z : {tcp_world[2]:>8.4f} m")

        # --- 4. FORCE & TORQUE DATA ---
        ft_data = rtde_r.getActualTCPForce()
        
        print("\n" + "="*50)
        print(" ⚖️ FORCE / TORQUE SENSOR (BASE FRAME)")
        print("="*50)
        print(f"Fx: {ft_data[0]:>8.2f} N   |  Tx: {ft_data[3]:>8.2f} Nm")
        print(f"Fy: {ft_data[1]:>8.2f} N   |  Ty: {ft_data[4]:>8.2f} Nm")
        print(f"Fz: {ft_data[2]:>8.2f} N   |  Tz: {ft_data[5]:>8.2f} Nm")
        print(f"Total Force Vector: {np.linalg.norm(ft_data[:3]):.2f} N")

        # --- 5. TOOL CONFIGURATION & SAFETY ---
        # FIX: getTCPOffset() belongs to the CONTROL interface
        active_tcp_offset = rtde_c.getTCPOffset() 
        payload = rtde_r.getPayload()
        safety_mode = rtde_r.getSafetyMode()
        
        print("\n" + "="*50)
        print(" 🛠️ HARDWARE CONFIGURATION & STATUS")
        print("="*50)
        print(f"Active TCP Offset (Z): {active_tcp_offset[2]:.4f} m")
        print(f"Registered Payload   : {payload:.2f} kg")
        print(f"Controller Safety Mode: {safety_mode} (1=Normal, 2=Reduced, 3=Protective Stop)")
        print("="*50 + "\n")

    except Exception as e:
        print(f"[ERROR] Failed to extract telemetry: {e}")
    finally:
        if 'robot' in locals() and robot is not None:
            robot.cleanup()
            print("[INFO] Connection safely closed.")

if __name__ == "__main__":
    print_telemetry()
