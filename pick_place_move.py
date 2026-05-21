import numpy as np
import ur_dual_controller as urcx
import time
from trajectory import Trajectory
import argparse
import pycurl
import xmlrpc.client
from io import BytesIO
import threading

# --- Configuration ---
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# --- Destination Poses ---
# The previous poses were unreachable and caused a safety stop.
# Using a standard, safe "home" pose for both arms to ensure success.
PICK_POSITION_POSE = [0.8399, 0.4085, -1.5899, -1.79, -0.844, 2.2184]
TAKE_POSITION_POSE = [0.0678, -2.0088, 2.2378, -2.5625, 4.6088, -0.0005]
TAKE_2_POSITION_POSE = [0.0702, -1.8146, 2.1715, -2.7095, 4.6033, -0.0006]

# --- Task-Specific Poses ---
GIVE_POSITION_POSE = [0.0418, -1.0165, -1.8723, -1.0687, 1.5364, 3.1638]
PLACE_POSITION_POSE = [-0.7148, -2.9587, 0.9348, -4.377, 5.4984, -2.2456]

# --- Home Poses ---
MASTER_HOME_POSE = [0.0004, 0.4938, -2.7899, -1.5941, -2.8716, 0.0268]
SLAVE_HOME_POSE = [0.0, -3.6373, 2.8053, -1.5785, 2.6658, 0.0002]



# --- Motion Parameters ---
MOVEMENT_TIME_SECONDS = 3.0 # Time for each robot's individual move. Lower is faster.
JOINT_ACCELERATION = 2.0    # Joint acceleration.
JOINT_VELOCITY = 3.0        # Joint velocity.
CONTROL_FREQUENCY_HZ = 125  # UR robot control frequency

class RG2:
    def __init__(self, robot_ip, rg_id):
        self.rg_id = rg_id
        self.robot_ip = robot_ip

    def get_rg_width(self):
        xml_request = f"""<?xml version="1.0"?>
    <methodCall>
        <methodName>rg_get_width</methodName>
            <params>
                <param>
                    <value><int>{self.rg_id}</int></value>
                </param>
            </params>
    </methodCall>"""

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        data = xml_request.replace('\r\n','').encode()

        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()

        xml_response = xmlrpc.client.loads(response.decode('utf-8'))
        rg_width = float(xml_response[0][0])
        #print(rg_width)
        return rg_width


    def rg_grip(self, target_width: float = 50, target_force: float= 10) -> bool:
        #assert target_width <= 100 and target_width >= 0, 'Target Width must be within the range [0,100]'
        #assert target_force <= 40 or target_force >= 0, 'Target force must be within the range [0,40]'

        # WARNING: params will be sent straight to electrical system with no error checking on robot!
        # if (target_width > 100):
        #     target_width = 100
        # if(target_width < 0):
        #     target_width = 0
        # if(target_force > 40):
        #     target_force = 40
        # if(target_force < 0):
        #     target_force = 0

        xml_request = f"""<?xml version="1.0"?>
    <methodCall>
    <methodName>rg_grip</methodName>
        <params>
            <param>
                <value><int>{self.rg_id}</int></value>
            </param>
            <param>
                <value><double>{target_width}</double></value>
            </param>
            <param>
                <value><double>{target_force}</double></value>
            </param>
        </params>
    </methodCall>"""

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        # headers = ["User-Agent: Python-PycURL", "Accept: application/json"]
        data = xml_request.replace('\r\n','').encode()
        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()

    def rg_open(self, target_force: float = 30.0):
        """Opens the gripper to maximum width."""
        self.rg_grip(target_width=100.0, target_force=target_force)

    def rg_close(self, target_force: float = 30.0):
        """Closes the gripper to minimum width."""
        self.rg_grip(target_width=54.2, target_force=target_force)

def execute_smooth_move(arm, dest_pose, movement_time, acc, vel, arm_name=""):
    """
    Moves a single robot arm from its current position to its destination
    pose using a smooth, generated trajectory. This is a blocking call.

    Args:
        arm: The URRobot object for the arm.
        dest_pose (list): The target joint configuration for the arm.
        movement_time (float): The time for the move in seconds.
        acc (float): The joint acceleration.
        vel (float): The joint velocity.
        arm_name (str): The name of the arm for logging purposes.
    """
    print(f"\n--- Moving {arm_name} Arm ---")

    # 1. Get current position
    print("[Step 1] Reading current arm position...")
    start_pose = np.array(arm.getj())
    print(f"✓ Start Pose:  {np.round(start_pose, 4).tolist()}")
    print(f"✓ Target Pose: {np.round(dest_pose, 4).tolist()}")

    # 2. Generate trajectory
    print("[Step 2] Generating smooth trajectory...")
    traj_generator = Trajectory(method="quintic")
    control_frequency = 1.0 / CONTROL_FREQUENCY_HZ

    path, _ = traj_generator.piecewise_interpolation(
        path=[start_pose, dest_pose],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    print(f"✓ Trajectory generated with {len(path)} waypoints.")

    # 3. Execute trajectory
    print("[Step 3] Executing trajectory...")
    arm_id = f"{arm_name.upper()} " if arm_name else ""
    print(f"!!! {arm_id}ARM WILL NOW MOVE !!!")

    arm.movejs(joint_positions_list=path, acc=acc, vel=vel, radius=0.01, wait=True)
    print(f"✓ {arm_name} arm movement complete.")


def execute_simultaneous_move(master_arm, master_dest_pose, slave_arm, slave_dest_pose, movement_time, acc, vel):
    """
    Moves both robot arms simultaneously to their destination poses.
    This is a blocking call that waits for both movements to finish.
    """
    print("\n--- Moving Both Arms Simultaneously ---")

    # 1. Get current positions
    master_start_pose = np.array(master_arm.getj())
    slave_start_pose = np.array(slave_arm.getj())
    print(f"✓ Master Start Pose: {np.round(master_start_pose, 4).tolist()}")
    print(f"✓ Slave Start Pose:  {np.round(slave_start_pose, 4).tolist()}")
    print(f"✓ Master Target Pose: {np.round(master_dest_pose, 4).tolist()}")
    print(f"✓ Slave Target Pose:  {np.round(slave_dest_pose, 4).tolist()}")

    # 2. Generate trajectories
    traj_generator = Trajectory(method="quintic")
    control_frequency = 1.0 / CONTROL_FREQUENCY_HZ

    master_path, _ = traj_generator.piecewise_interpolation(
        path=[master_start_pose, master_dest_pose],
        control_frequency=control_frequency,
        interval_time=movement_time
    )

    slave_path, _ = traj_generator.piecewise_interpolation(
        path=[slave_start_pose, slave_dest_pose],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    print(f"✓ Trajectories generated.")

    # 3. Execute trajectories without waiting
    print("[Step 3] Executing simultaneous trajectory...")
    print("!!! BOTH ARMS WILL NOW MOVE !!!")
    master_arm.movejs(joint_positions_list=master_path, acc=acc, vel=vel, radius=0.01, wait=False)
    slave_arm.movejs(joint_positions_list=slave_path, acc=acc, vel=vel, radius=0.01, wait=False)

    # 4. Wait for both movements to complete
    while master_arm.is_program_running() or slave_arm.is_program_running():
        time.sleep(0.05)

    print("✓ Simultaneous movement complete.")


def main():
    """Main function to run the sequential smooth move program."""
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="Move UR robots sequentially to destination and/or home poses.")
    parser.add_argument("--home", action="store_true", help="Only move the robots to their home positions.")
    args = parser.parse_args()

    print("=" * 60)
    print("Starting Sequential Smooth Move Program")
    print("=" * 60)
    print("WARNING: This script will move the robots. Press Ctrl+C to stop.")
    print("=" * 60)

    rbtx = None
    try:
        # Initialize the dual controller
        print("\n[INIT] Initializing URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        master_arm = rbtx._lft_arm
        slave_arm = rbtx._rgt_arm

        if args.home:
            # --- Go directly to Home ---
            print("\n" + "="*60)
            print("  MOVING MASTER TO GIVE POSITION")
            print("="*60 + "\n")
            execute_smooth_move(
                master_arm, GIVE_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Master"
            )
            print("\n" + "="*60)
            print("  MOVING SLAVE TO PLACE POSITION")
            print("="*60 + "\n")
            execute_smooth_move(
                slave_arm, PLACE_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Slave"
            )

        else: # Covers --full-sequence and default case
            # --- Master Arm: Pick and Give Sequence ---
            print("\n" + "="*60)
            print("Part 1: Master Arm - Pick Operation")
            print("="*60)
            execute_smooth_move(
                master_arm, PICK_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Master"
            )

            # --- Manual Gripper Control ---
            print("\n" + "="*60)
            print("MANUAL GRIPPER CONTROL FOR MASTER ARM")
            print("="*60)
            
            master_gripper = RG2(MASTER_ROBOT_IP, 0) # rg_id is 0
            
            print(f"Master gripper initial width: {master_gripper.get_rg_width()}")
            
            while True:
                command_str = input("Enter command for master gripper ('open', 'close', or 'exit'): ").lower().strip()
                
                if command_str == 'exit':
                    break
        
                if command_str == 'open':
                    master_gripper.rg_open()
                    print("Master gripper open command sent.")
                elif command_str == 'close':
                    master_gripper.rg_close()
                    print("Master gripper close command sent.")
                else:
                    print("Invalid command. Use 'open', 'close', or 'exit'.")

            print("\nResuming robot sequence...")

            print("\n" + "="*60)
            print("Part 2: Master Arm - Give Operation")
            print("="*60)
            execute_smooth_move(
                master_arm, GIVE_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Master"
            )

            print("\nWaiting for 1 second...")
            time.sleep(1)

            # --- Slave Arm: Move to Destination ---
            print("\n" + "="*60)
            print("Part 3: Slave Arm - Move to Destination")
            print("="*60)
            execute_smooth_move(
                slave_arm, TAKE_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Slave"
            )

            print("\nWaiting for 1 second...")
            time.sleep(1)

            # --- Slave Arm: Move to Take 2 Position ---
            print("\n" + "="*60)
            print("Part 3.1: Slave Arm - Move to Take 2 Position")
            print("="*60)
            execute_smooth_move(
                slave_arm, TAKE_2_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Slave"
            )


            print("\nWaiting for 2 seconds before handoff...")
            time.sleep(2)

            # --- Sequential Gripper Handoff ---
            print("\n" + "="*60)
            print("Part 3.5: Gripper Handoff")
            print("="*60)

            slave_gripper = RG2(SLAVE_ROBOT_IP, 0) # rg_id is 0

            print("!!! SLAVE GRIPPER CLOSING TO TAKE OBJECT !!!")
            # Close to the specified width to grip the object
            slave_gripper.rg_grip(target_width=55.5, target_force=30.0)
            print("✓ Slave gripper close command sent.")

            print("\nWaiting 1 second...")
            time.sleep(1)

            print("\n!!! MASTER GRIPPER OPENING TO RELEASE OBJECT !!!")
            # Open to the specified width for a controlled release
            master_gripper.rg_grip(target_width=100.0, target_force=30.0)
            print("✓ Master gripper open command sent.")

            print("\n✓ Gripper handoff complete.")


            print("\nWaiting for 2 seconds after handoff...")
            time.sleep(2)

            print("\n" + "="*60)
            print("Part 4: Slave Arm - Place Operation")
            print("="*60)
            execute_smooth_move(
                slave_arm, PLACE_POSITION_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY, "Slave"
            )

            print("\nOpening slave gripper...")
            slave_gripper.rg_open()
            print("✓ Slave gripper open command sent.")

            print("\nWaiting for 2 seconds...")
            time.sleep(2)

            # --- Both Arms: Return to Home ---
            print("\n" + "="*60)
            print("Part 5: Both Arms - Return to Home")
            print("="*60)
            execute_simultaneous_move(
                master_arm, MASTER_HOME_POSE,
                slave_arm, SLAVE_HOME_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # Cleanly close the connections
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    main() 