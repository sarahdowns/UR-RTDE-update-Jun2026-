import os
import struct
import socket
import program_builder as pb
import numpy as np
import ur_robot
import trajectory as traj


class URDualController(object):

    def __init__(self, master_robot_ip='192.168.5.4', slave_robot_ip='192.168.5.5', control_pc_ip='192.168.5.1'):
        # left arm

        self._lft_arm = ur_robot.URRobot(master_robot_ip)
        self._rgt_arm = ur_robot.URRobot(slave_robot_ip)
        # setup control pc server
        self._pc_server_socket_addr = (control_pc_ip, 0)  # 0: the system finds an available port
        self._pc_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._pc_server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._pc_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._pc_server_socket.bind(self._pc_server_socket_addr)
        self._pc_server_socket.listen(5)
        self._jnts_scaler = 1e6
        self._pb = pb.ProgramBuilder()
        self._script_dir = os.path.dirname(__file__)
        self._pb.load_prog(os.path.join(self._script_dir, "urscript/moderndriver_master.script"))
        self._master_modern_driver_urscript = self._pb.get_program_to_run()
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_pc_ip",
                                                                                          self._pc_server_socket.getsockname()[
                                                                                              0])
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_pc_port",
                                                                                          str(
                                                                                              self._pc_server_socket.getsockname()[
                                                                                                  1]))
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_slave_ip",
                                                                                          slave_robot_ip)
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_jnts_scaler",
                                                                                          str(self._jnts_scaler))
        self._pb.load_prog(os.path.join(self._script_dir, "urscript/moderndriver_slave.script"))
        self._slave_modern_driver_urscript = self._pb.get_program_to_run()
        self._slave_modern_driver_urscript = self._slave_modern_driver_urscript.replace("parameter_master_ip",
                                                                                        master_robot_ip)
        self._slave_modern_driver_urscript = self._slave_modern_driver_urscript.replace("parameter_jnts_scaler",
                                                                                        str(self._jnts_scaler))
        self._trajt = traj.Trajectory(method='quintic')

    def move_jntspace_path(self, path, control_frequency=.008, interval_time=1.0, interpolation_method=None):
        
        self._trajt.set_interpolation_method(interpolation_method)
        interpolated_confs, interpolated_spds = self._trajt.piecewise_interpolation(path, control_frequency,
                                                                                     interval_time)
        # upload a urscript to connect to the pc server started by this class
        self._lft_arm.send_program(self._master_modern_driver_urscript)
        self._rgt_arm.send_program(self._slave_modern_driver_urscript)
        # accept arm socket
        pc_server_socket, pc_server_socket_addr = self._pc_server_socket.accept()
        print("PC server connected by ", pc_server_socket_addr)
        # send trajectory
        keepalive = 1
        buf = bytes()
        for id, conf in enumerate(interpolated_confs):
            if id == len(interpolated_confs) - 1:
                keepalive = 0
            jointsradint = [int(jnt_value * self._jnts_scaler) for jnt_value in conf]
            buf += struct.pack('!iiiiiiiiiiiii', jointsradint[0], jointsradint[1], jointsradint[2],
                               jointsradint[3], jointsradint[4], jointsradint[5], jointsradint[6],
                               jointsradint[7], jointsradint[8], jointsradint[9], jointsradint[10],
                               jointsradint[11], keepalive)
        pc_server_socket.send(buf)
        pc_server_socket.close()

    def get_jnt_values(self):
       
        return np.array(self._lft_arm.getj() + self._rgt_arm.getj())

if __name__ == '__main__':
    # This block will only run when you execute this script directly
    # It allows you to test the functionality of the URDualController class

    print("=" * 60)
    print("Running URDualController Test")
    print("=" * 60)
    
    # Use the default IPs defined in the __init__ method
    # Make sure they match your setup
    master_ip = "192.168.5.4"
    slave_ip = "192.168.5.5"
    pc_ip = "192.168.5.1"

    print(f"Attempting to connect to:")
    print(f"  Master Robot: {master_ip}")
    print(f"  Slave Robot:  {slave_ip}")
    print(f"  Control PC:   {pc_ip}")

    try:
        # 1. Initialize the controller
        dual_controller = URDualController(
            master_robot_ip=master_ip,
            slave_robot_ip=slave_ip,
            control_pc_ip=pc_ip
        )
        print("\n✓ Successfully initialized URDualController.")

        # 2. Get and print the joint values
        print("\nAttempting to read joint values from both robots...")
        current_joints = dual_controller.get_jnt_values()
        
        print("✓ Successfully retrieved joint values.")
        print("\n--- Current Joint Angles (radians) ---")
        print(f"Master Arm (Joints 0-5):  {np.round(current_joints[:6], 4)}")
        print(f"Slave Arm (Joints 6-11):  {np.round(current_joints[6:], 4)}")
        print("------------------------------------")

    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
        print("Please check the following:")
        print("  - Are both robots powered on and not in an error state?")
        print("  - Is the network connection stable?")
        print("  - Are the IP addresses correct?")
    
    finally:
        # 3. Cleanly close the connections
        if 'dual_controller' in locals():
            dual_controller._lft_arm.close()
            dual_controller._rgt_arm.close()
            print("\n✓ Robot connections closed.")
            
    print("\nTest finished.")
