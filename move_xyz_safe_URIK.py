# File name: move_xyz_safe_URIK.py
# Same as move_xyz_safe.py but uses UR's built-in IK instead of Corke's (toolbox) numerical IK. Maintains the elbow safety check 
# and TCP offsets, but rely on the robot to calculate joint angles internally.

### UPDATE APRIL 6, 2026: DOESN'T WORK YET WITH test_move_setpoint.py

from spatialmath import SE3
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import numpy as np
import roboticstoolbox as rtb  # optional, only for visualization

class UR5eSafeController_URIK:
    def __init__(self, ip="192.168.5.5"):
        self.rtde_c = RTDEControl(ip)       # Initialize RTDE Interfaces
        self.rtde_r = RTDEReceive(ip)
        
        # Update the last link (d6) to include the gripper length
        # tool_offset = 0.23 		# meters RG2
        tool_offset = 0.30 		# meters RG6
        total_d6 = 0.0996 + tool_offset  # Table d6 is 0.0996m
        
        # Optional visualization model
        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(alpha=np.pi/2,  a=0,       d=0.1625), 
            rtb.RevoluteDH(alpha=0,        a=-0.425,  d=0),     
            rtb.RevoluteDH(alpha=0,        a=-0.3922, d=0),     
            rtb.RevoluteDH(alpha=np.pi/2,  a=0,       d=0.1333), 
            rtb.RevoluteDH(alpha=-np.pi/2, a=0,       d=0.0997), 
            rtb.RevoluteDH(alpha=0,        a=0,       d=total_d6)
        ], name="UR5_Custom_Match")
        self.robot.base = SE3.Ry(np.deg2rad(45))

    def move_to_xyz_safe(self, x, y, z, pitch_deg=180, speed=0.25, acceleration=0.5, visualize=True):
        """
        Move UR5 to target XYZ using UR's built-in IK (via movel).
        pitch_deg: gripper tilt in degrees
        """
        if visualize:
            self.robot.plot([0,0,0,0,0,0])  # optional visualization

        # Prompt user
        if input(f"Move to ({x:.3f}, {y:.3f}, {z:.3f})? (y/n): ").lower() != 'y':
            print("Move cancelled.")
            return False

        # Convert pitch to radians
        pitch_rad = np.deg2rad(pitch_deg)

        # Build URScript pose as [X,Y,Z,Rx,Ry,Rz]
        # Here Rx,Ry,Rz is rotation vector representation of orientation
        # For a straight-down pitch, we rotate around Y
        # This is an approximation: straight down with pitch_deg rotation
        rx = 0.0
        ry = pitch_rad
        rz = 0.0
        pose_list = [x, y, z, rx, ry, rz]

        try:
            # Send URScript movel command (uses UR IK internally)
            self.rtde_c.sendCustomScript(f"movel(p{pose_list}, a={acceleration}, v={speed})")

            # Optional: check elbow angle after motion
            actual_q = self.rtde_r.getActualQ()
            elbow_angle_deg = np.rad2deg(actual_q[2])
            if abs(elbow_angle_deg) < 10:
                print(f"[WARNING] Elbow angle too small: {elbow_angle_deg:.1f}°")

            print("✅ Motion executed successfully.")
            return True

        except Exception as e:
            print(f"[ERROR] Motion failed: {e}")
            return False

    def cleanup(self):
        """Safely stop RTDE scripts."""
        self.rtde_c.stopScript()
        print("🔌 RTDE connection closed.")
