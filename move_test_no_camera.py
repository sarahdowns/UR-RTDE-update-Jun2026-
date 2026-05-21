# File: move_test_no_camera.py

from move_xyz_safe import UR5eSafeController

def manual_jog():
    # Initialize the controller
    ctrl = UR5eSafeController(ip="192.168.5.5")
    
    print("\n=== Manual Jogging Utility ===")
    print("Enter target coordinates to test kinematic accuracy.")
    
    try:
        while True:
            try:
                x = float(input("Enter X (meters) [or q to quit]: "))
                y = float(input("Enter Y (meters): "))
                z = float(input("Enter Z (meters): "))
                pitch = float(input("Enter approach pitch (deg, default 180): ") or 180)
                
                # Execute the move using your calibrated method
                success = ctrl.move_to_xyz_safe(x, y, z, pitch_deg=pitch)
                
                if not success:
                    print("[RESULT] Movement failed or aborted.")
            
            except ValueError:
                print("Exiting...")
                break
                
    finally:
        ctrl.cleanup()

if __name__ == "__main__":
    manual_jog()
