# File name: move_xyz_safe.py
# Best singularity avoidance code as of April 2026

import numpy as np
import pandas as pd
import roboticstoolbox as rtb		# Corke toolbox
import socket
from spatialmath import SE3
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

class UR5eSafeController:
    def __init__(self, ip):
        # Initialize RTDE Interfaces
        self.ip = ip
        self.rtde_c = RTDEControl(ip)
        self.rtde_r = RTDEReceive(ip)
        active_tcp = self.rtde_c.getTCPOffset()
        
        # Update the last link (d6) to include the gripper length. Check configuration on teach pendant
        # tool_offset = 0.23 		# meters RG2
        # tool_offset = 0.30 		# meters RG6
        print(f"[INIT] Dynamic Tool Offset Extracted: Z = {active_tcp[2]:.4f} m")
        # total_d6 = 0.0996 + tool_offset # Table d6 is 0.0996m
        
        self.robot = rtb.DHRobot([
            # alpha, a, d
            rtb.RevoluteDH(alpha=np.pi/2,  a=0,       d=0.1625), 
            rtb.RevoluteDH(alpha=0,        a=-0.425,  d=0),     
            rtb.RevoluteDH(alpha=0,        a=-0.3922, d=0),     
            rtb.RevoluteDH(alpha=np.pi/2,  a=0,       d=0.1333), 
            rtb.RevoluteDH(alpha=-np.pi/2, a=0,       d=0.0997), 
            rtb.RevoluteDH(alpha=0,        a=0,       d=0.0996)
        ], name="UR5_Custom_Match")  # mdh to use modified DH
        
        # Apply 45-degree base tilt relative to gravity (-45 for 192.168.5.4) and configured offset
        self.robot.base = SE3.Ry(np.deg2rad(45))
        self.robot.tool = SE3(active_tcp[0], active_tcp[1], active_tcp[2]) * SE3.RPY(active_tcp[3], active_tcp[4], active_tcp[5])

    def move_to_xyz_safe(self, x, y, z, visualize=True, speed=0.3, acceleration=1.5, roll_deg=0, pitch_deg=0, yaw_deg=0):
        home_q = self.rtde_r.getActualQ()
        
        # Convert all degrees to radians for full 3D spatial rotation
        r_rad = np.deg2rad(roll_deg)
        p_rad = np.deg2rad(pitch_deg)
        y_rad = np.deg2rad(yaw_deg)
        
        # SE3.RPY applies rotations in Roll (X), Pitch (Y), Yaw (Z)
        T_target = SE3(x, y, z) * SE3.RPY(r_rad, p_rad, y_rad)
        
        # Forward-extended seed to prevent 'wrapped' configurations
        safe_seed = [0, -np.pi/3, -np.pi/2, -np.pi/2, np.pi/2, 0]
        
        # --- MASKING ---
        # Weight mask: [X, Y, Z, Roll, Pitch, Yaw]
        # 1.0 = Strict requirement, 0.1 = Flexible/Allowed to tilt
        W = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
        
        sol = self.robot.ikine_LM(T_target, q0=home_q, mask=W)	# Solve Inverse Kinematics with the mask

        if sol.success:
            # Check for Elbow Safety (Joint 3)
            if abs(sol.q[2]) < np.deg2rad(10):
                print(f"[DANGER] Rejected: Elbow ({np.rad2deg(sol.q[2]):.1f}°) too close to self-collision.")
                return False

            # Display the actual achieved pose vs target
            achieved_pose = self.robot.fkine(sol.q)			# Forward Kinematics
            pos_error = np.linalg.norm(achieved_pose.t - T_target.t)
            
            print(f"--- IK Success ---")
            print(f"Position Error: {pos_error*1000:.2f} mm")
            print(f"Joints (deg): {np.rad2deg(sol.q).round(1)}")
            #print(f"TCP (cm): X: {achieved_pose[0]*100:.1f}, Y: {achieved_pose[1]*100:.1f}, Z: {achieved_pose[2]*100:.1f}")

            if visualize:
                print("[VISUAL] Confirm the path in the plot window.")
                self.robot.plot(sol.q, block=True)			# Corke's built in visulaization
            
            if input(f"Move to ({x:.3f}, {y:.3f}, {z:.3f})? (y/n): ").lower() == 'y':
                self.rtde_c.moveJ(sol.q, speed, acceleration)
                return True
        else:
            dist = np.linalg.norm([x, y, z])
            print(f"[ERROR] Reach unreachable even with tilt. Dist: {dist:.3f}m")
            return False

    ### Force/Torque Sensor ###
    def get_ft_sensor_data(self):
        """
        Returns the 6-axis force/torque sensor readings.
        Format: [Fx, Fy, Fz, Tx, Ty, Tz]
        Units: Newtons and Newton-meters
        """
        return self.rtde_r.getActualTCPForce()

    def zero_ft_sensor(self):
        """
        Resets the F/T sensor to zero. 
        Note: The robot must be stationary when calling this.
        """
        # This sends a URScript command to the controller to tare the sensor
        self.rtde_c.zeroFtSensor()
        print("[INFO] F/T Sensor zeroed.")
        
    def get_ft_sensor_baserot(self, world_frame=True):
        """
        Returns the 6-axis force/torque sensor readings.
        If world_frame=True, rotates the data to account for the -45° base tilt.
        """
        # 1. Get raw data from robot (returned in Robot Base Frame)
        raw_ft = np.array(self.rtde_r.getActualTCPForce())
        
        if not world_frame:
            return raw_ft

        # 2. Extract the 3x3 Rotation Matrix from your existing base setup
        # self.robot.base is the SE3 object; .R gives the rotation part
        R_base = self.robot.base.R 
        
        # 3. Split 6-axis vector into Force and Torque
        f_robot = raw_ft[:3]
        t_robot = raw_ft[3:]
        
        # 4. Rotate into World Frame: F_world = R_base * F_robot
        f_world = R_base @ f_robot
        t_world = R_base @ t_robot
        
        # 5. Recombine into the standard 6-element list
        return np.concatenate([f_world, t_world])
        
    ### Reset Protective Stop (ENABLE WITH CAUTION) ###
    def reset_protective_stop(self):
        """
        Connects to the Dashboard Server (Port 29999) to unlock the robot.
        """
        try:
            # Dashboard Server uses standard TCP sockets
            dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            dashboard.settimeout(2.0)
            dashboard.connect((self.ip, 29999))
            
            # Flush the initial 'Connected' message
            dashboard.recv(1024)
            
            # 1. Attempt to unlock
            dashboard.sendall(b"unlock protective stop\n")
            # Decode and strip to handle any trailing newlines/spaces
            response = dashboard.recv(1024).decode().strip()
            
            # 2. Check the specific status
            if "No protective stop to unlock" in response:
                print("no protective stop detected")
                dashboard.close()
                return True
                
            elif "Protective stop released" in response:
                print("--- Safety Reset Success ---")
                # Only close the popup if there was actually a stop to clear
                dashboard.sendall(b"close safety popup\n")
                dashboard.recv(1024)
                dashboard.close()
                return True
            
            else:
                # This catches other states (e.g., if the E-Stop is physically pressed)
                print(f"[WARNING] Dashboard response: {response}")
                dashboard.close()
                return False
                
        except Exception as e:
            print(f"[ERROR] Could not connect to Dashboard: {e}")
            return False
                
    def cleanup(self):
        """Safely close RTDE connections."""
        self.rtde_c.stopScript()
