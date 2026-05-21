import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

def create_sim_robot():
    tool_offset = 0.191
    total_d6 = 0.0996 + tool_offset 
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=0.1625, a=0,      alpha=np.pi/2,  qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=0,      a=-0.425,  alpha=0,        qlim=[-np.pi, 0]), 
        rtb.RevoluteDH(d=0,      a=-0.3922, alpha=0,        qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=0.1333, a=0,      alpha=np.pi/2,  qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=0.0997, a=0,      alpha=-np.pi/2, qlim=[-np.pi, np.pi]),
        rtb.RevoluteDH(d=total_d6, a=0,    alpha=0,        qlim=[-np.pi, np.pi])
    ], name="UR5e_Sim")

    # The 45-degree base tilt
    robot.base = SE3.Ry(np.deg2rad(45))
    return robot

def run_pickup_simulation(ball_cam_xyz):
    robot = create_sim_robot()
    
    # Try to load real calibration
    try:
        T_cam_to_robot = np.load("cam_to_robot_transform.npy")
        print("[INFO] Using real calibration matrix.")
    except:
        T_cam_to_robot = np.eye(4)
        print("[WARNING] Using Identity Matrix (Camera = Robot Base).")

    # Transform coordinates
    ball_pos_cam = np.array([ball_cam_xyz[0], ball_cam_xyz[1], ball_cam_xyz[2], 1.0])
    ball_pos_robot = (T_cam_to_robot @ ball_pos_cam)[:3]
    
    print(f"\n--- COORDINATE DIAGNOSTICS ---")
    print(f"Target in World XYZ (relative to floor): {ball_pos_robot}")
    
    # Calculate reach distance from base origin
    dist_from_base = np.linalg.norm(ball_pos_robot)
    print(f"Distance from Robot Base Origin: {dist_from_base:.3f}m")

    # Calculate targets
    SAFE_HOVER_Z = max(ball_pos_robot[2] + 0.15, 0.25)
    T_hover = SE3(ball_pos_robot[0], ball_pos_robot[1], SAFE_HOVER_Z) * SE3.RPY(0, np.pi, 0)

    # --- TOLERANT IK SOLVER ---
    # Weight mask: [X, Y, Z, Roll, Pitch, Yaw]
    # We give 1.0 to position, but reduce orientation weights to 0.1
    # This allows the robot to tilt the gripper if it helps reach the XYZ
    W = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
    
    safe_seed = [0, -np.pi/3, -np.pi/2, -np.pi/2, np.pi/2, 0]
    
    # solve using Levenberg-Marquardt with the weight mask
    sol_hover = robot.ikine_LM(T_hover, q0=safe_seed, mask=W)
    
    if sol_hover.success:
        print("[SUCCESS] Position found (may be slightly tilted).")
        # Check the error
        actual_pose = robot.fkine(sol_hover.q)
        pos_error = np.linalg.norm(actual_pose.t - T_hover.t)
        print(f"Position Error: {pos_error*1000:.2f} mm")
        
        robot.plot(sol_hover.q, block=True)
    else:
        print("[FAIL] Even with relaxed orientation, the point is unreachable.")
        print(f"Attempted World XYZ: {ball_pos_robot[0]:.3f}, {ball_pos_robot[1]:.3f}, {SAFE_HOVER_Z:.3f}")

if __name__ == "__main__":
    # Test coordinates
    test_ball = [0.1, 0.0, 0.4]
    run_pickup_simulation(test_ball)
