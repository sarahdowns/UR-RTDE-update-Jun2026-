# Author: Sarah Downs
# Move robot arms to a position above the table and receive coordinates from the
# zed camera, pick up object.
# Edited to synchronize robots with a Modbus signal from Robot 1.

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from gripper_RG2 import RG2
import time
from pyModbusTCP.client import ModbusClient
import threading

# --- Robot and Gripper Configuration ---
rg_id = 0
robot1_ip = "192.168.5.5"  # Robot 1 (Signaling Robot, hosts gripper)
robot2_ip = "192.168.5.4"  # Robot 2
robot3_ip = "192.168.5.3"  # Robot 3

# --- Modbus Configuration ---
MODBUS_PORT = 502

# Define Modbus Register Addresses (Adjust as needed for your application)
# These are holding registers (usually starting from address 0 or 1 for UR)
ROBOT1_SIGNAL_REGISTER = 100 # Robot 1 will write to this, Robots 2 & 3 will read

# --- RTDE Control and Receive Interfaces ---
# Robot 1 interfaces
rtde_c1 = RTDEControl(robot1_ip)
rtde_r1 = RTDEReceive(robot1_ip)
rg_gripper = RG2(robot1_ip, rg_id) # Gripper is on Robot 1

# Robot 2 interfaces
rtde_c2 = RTDEControl(robot2_ip)
rtde_r2 = RTDEReceive(robot2_ip)

# Robot 3 interfaces
rtde_c3 = RTDEControl(robot3_ip)
rtde_r3 = RTDEReceive(robot3_ip)

# --- Modbus Clients ---
# Modbus client for Robot 1 (for Robot 1 to write its own signals)
modbus_client_r1 = ModbusClient(host=robot1_ip, port=MODBUS_PORT, auto_open=True, auto_close=False)
# Modbus client for Robot 2 (can write its own signals if needed later, but here mainly for control)
modbus_client_r2 = ModbusClient(host=robot2_ip, port=MODBUS_PORT, auto_open=True, auto_close=False)
# Modbus client for Robot 3 (can write its own signals if needed later)
modbus_client_r3 = ModbusClient(host=robot3_ip, port=MODBUS_PORT, auto_open=True, auto_close=False)

# --- Initial Robot Positions ---
init_q1 = rtde_r1.getActualQ()
print(f"Robot 1 Initial position: {init_q1}")
init_q2 = rtde_r2.getActualQ()
print(f"Robot 2 Initial position: {init_q2}")
init_q3 = rtde_r3.getActualQ()
print(f"Robot 3 Initial position: {init_q3}")

# --- Global Robot Parameters (can be adjusted per robot if needed) ---
vel = 0.5
acc = 2.0
blend = 0.099

# --- Placeholder for Camera Data (replace with actual ZED camera integration) ---
cam_posX = -0.15
cam_posY = 0.25
cam_posZ = 0.25
cam_rx = -1.401
cam_ry = 0.406
cam_rz = 0.746

# --- Placeholder for Robot 2 & 3 Forward Positions ---
# IMPORTANT: Replace these with your actual desired forward joint positions
robot2_forward_position = [0.1, -1.2, 1.8, -2.2, -1.0, 0.0, vel, acc, blend]
robot3_forward_position = [-0.1, -1.0, 1.5, -2.0, -1.5, 0.5, vel, acc, blend]

# --- Robot Control Functions ---

def control_robot_1(rtde_c, rtde_r, rg_gripper, modbus_client_for_r1_signal):
    """Controls Robot 1: Moves, grips, and signals completion via Modbus."""
    print("--- Robot 1 (Signaling Robot): Starting sequence ---")
    try:
        # Move to initial pick position (existing logic)
        path1_pre_grip = Path()
        path1_pre_grip.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.1008, 0.186, 0.376, -1.401, 0.406, 0.746, vel, acc, 0.0]))
        path1_pre_grip.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.2016, 0.196, 0.644, -1.401, 0.406, 0.746, vel, acc, 0.0]))
        
        print("Robot 1: Moving to pre-grip position...")
        rtde_c.movePath(path1_pre_grip, True)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)

        # Move to camera-detected object position
        cam_grip_path = Path()
        cam_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [cam_posX, cam_posY, cam_posZ, cam_rx, cam_ry, cam_rz, vel, acc, 0.0]))
        print("Robot 1: Moving to camera position to pick...")
        rtde_c.movePath(cam_grip_path, True)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)

        # Perform gripping action
        target_force = 30.00
        print("Robot 1: Gripping object...")
        rg_gripper.rg_grip(100, target_force)
        time.sleep(1) # Give gripper time to close
        rg_gripper.rg_grip(50, target_force) # Adjust to hold securely
        time.sleep(1)

        # Lift object
        post_grip_path = Path()
        post_grip_path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, [1.0, -2.0, 2.0, -1.5263, -2.8569, 0.167, vel, acc, blend]))
        print("Robot 1: Lifting object...")
        rtde_c.movePath(post_grip_path, True)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)
        
        # --- Modbus Signal to other robots ---
        if modbus_client_for_r1_signal.is_open():
            # Set the signal register to 1 (True) for other robots to read
            modbus_client_for_r1_signal.write_single_register(ROBOT1_SIGNAL_REGISTER, 1)
            print("Robot 1: Sent 'GO' signal (1) via Modbus to register", ROBOT1_SIGNAL_REGISTER)
        else:
            print("Robot 1 Modbus client not connected. Cannot send signal.")

        # Return to initial position (or move to drop-off point)
        print("Robot 1: Returning to initial position.")
        rtde_c.moveJ(init_q1)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)
        print("Robot 1: Sequence finished.")

    except Exception as e:
        print(f"Error in Robot 1 control: {e}")
    finally:
        # Ensure RTDE script is stopped
        rtde_c.stopScript()


def control_robot_2(rtde_c, rtde_r, modbus_client_for_r2_itself, modbus_client_to_read_r1_signal):
    """Controls Robot 2: Waits for Robot 1's signal, moves forward, then returns."""
    print("--- Robot 2: Starting sequence ---")
    try:
        # --- Wait for signal from Robot 1 ---
        print("Robot 2: Waiting for signal from Robot 1 (register", ROBOT1_SIGNAL_REGISTER, ")...")
        signal_received = 0
        while signal_received == 0:
            if modbus_client_to_read_r1_signal.is_open():
                holding_registers = modbus_client_to_read_r1_signal.read_holding_registers(ROBOT1_SIGNAL_REGISTER, 1)
                if holding_registers is not None and len(holding_registers) > 0:
                    signal_received = holding_registers[0]
                else:
                    print("Robot 2: Could not read Robot 1's signal register. Retrying...")
            else:
                print("Robot 2: Modbus client to read Robot 1's signal not connected. Attempting to reconnect...")
                modbus_client_to_read_r1_signal.open() # Try to re-open if closed
            time.sleep(0.5) # Wait before retrying
        print("Robot 2: Received signal from Robot 1!")

        # --- Move forward ---
        path2_forward = Path()
        path2_forward.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, robot2_forward_position))
        
        print("Robot 2: Moving forward...")
        rtde_c.movePath(path2_forward, True)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)
        
        # Simulate some work or hold position
        time.sleep(2) 

        # --- Move back to initial position ---
        print("Robot 2: Moving back to initial position.")
        rtde_c.moveJ(init_q2)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)
        
        print("Robot 2: Sequence finished.")

    except Exception as e:
        print(f"Error in Robot 2 control: {e}")
    finally:
        rtde_c.stopScript()


def control_robot_3(rtde_c, rtde_r, modbus_client_for_r3_itself, modbus_client_to_read_r1_signal):
    """Controls Robot 3: Waits for Robot 1's signal, moves forward, then returns."""
    print("--- Robot 3: Starting sequence ---")
    try:
        # --- Wait for signal from Robot 1 ---
        print("Robot 3: Waiting for signal from Robot 1 (register", ROBOT1_SIGNAL_REGISTER, ")...")
        signal_received = 0
        while signal_received == 0:
            if modbus_client_to_read_r1_signal.is_open():
                holding_registers = modbus_client_to_read_r1_signal.read_holding_registers(ROBOT1_SIGNAL_REGISTER, 1)
                if holding_registers is not None and len(holding_registers) > 0:
                    signal_received = holding_registers[0]
                else:
                    print("Robot 3: Could not read Robot 1's signal register. Retrying...")
            else:
                print("Robot 3: Modbus client to read Robot 1's signal not connected. Attempting to reconnect...")
                modbus_client_to_read_r1_signal.open() # Try to re-open if closed
            time.sleep(0.5) # Wait before retrying
        print("Robot 3: Received signal from Robot 1!")

        # --- Move forward ---
        path3_forward = Path()
        path3_forward.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionJoints, robot3_forward_position))
        
        print("Robot 3: Moving forward...")
        rtde_c.movePath(path3_forward, True)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)
        
        # Simulate some work or hold position
        time.sleep(2) 

        # --- Move back to initial position ---
        print("Robot 3: Moving back to initial position.")
        rtde_c.moveJ(init_q3)
        while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
            time.sleep(0.1)
        
        print("Robot 3: Sequence finished.")

    except Exception as e:
        print(f"Error in Robot 3 control: {e}")
    finally:
        rtde_c.stopScript()

# --- Main Execution ---
if __name__ == "__main__":
    print("Starting multi-robot control script...")

    # Ensure Modbus clients are open (auto_open=True should handle this, but good to check)
    modbus_clients = [modbus_client_r1, modbus_client_r2, modbus_client_r3]
    for client in modbus_clients:
        if not client.is_open():
            if not client.open():
                print(f"Failed to connect to Modbus client for {client.host}.")

    # --- Reset Modbus registers for a clean start ---
    # This is crucial for repetitive testing. Robot 1's signal register needs to be reset to 0.
    try:
        if modbus_client_r1.is_open():
            modbus_client_r1.write_single_register(ROBOT1_SIGNAL_REGISTER, 0)
            print(f"Modbus register {ROBOT1_SIGNAL_REGISTER} on {robot1_ip} reset to 0.")
        else:
            print(f"Cannot reset Modbus register on {robot1_ip}: client not open.")
    except Exception as e:
        print(f"Error resetting Modbus register: {e}")
        
    # Create threads for each robot
    # Pass modbus_client_r1 to R2 and R3 as the client they will use to READ Robot 1's signal
    robot1_thread = threading.Thread(target=control_robot_1, args=(rtde_c1, rtde_r1, rg_gripper, modbus_client_r1))
    robot2_thread = threading.Thread(target=control_robot_2, args=(rtde_c2, rtde_r2, modbus_client_r2, modbus_client_r1))
    robot3_thread = threading.Thread(target=control_robot_3, args=(rtde_c3, rtde_r3, modbus_client_r3, modbus_client_r1))

    # Start the threads
    # Robot 1 is started first as it sends the signal
    robot1_thread.start()
    robot2_thread.start() # Robot 2 will immediately start waiting
    robot3_thread.start() # Robot 3 will immediately start waiting

    # Wait for all threads to complete
    robot1_thread.join()
    robot2_thread.join()
    robot3_thread.join()

    print("All robot sequences completed.")

    # Close RTDE connections
    rtde_c1.disconnect()
    rtde_r1.disconnect()
    rtde_c2.disconnect()
    rtde_r2.disconnect()
    rtde_c3.disconnect()
    rtde_r3.disconnect()

    # Close Modbus connections
    for client in modbus_clients:
        if client.is_open():
            client.close()
    print("RTDE and Modbus connections closed.")