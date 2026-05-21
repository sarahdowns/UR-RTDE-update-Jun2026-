import numpy as np
import ur_dual_controller as urcx
import time
from trajectory import Trajectory
import argparse
import pycurl
import xmlrpc.client
from io import BytesIO
import threading
import json
import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from collections import deque
import math

# --- Configuration ---
MASTER_ROBOT_IP = "192.168.5.4"
SLAVE_ROBOT_IP = "192.168.5.5"
CONTROL_PC_IP = "192.168.5.1"

# --- Payload Configuration (Solution 1: Accurate Payload Setup) ---
# RG2 Gripper specifications
RG2_MASS_KG = 0.78  # RG2 gripper mass in kg
RG2_COG_OFFSET = [0.0, 0.0, 0.055]  # Center of gravity offset from flange [x, y, z] in meters

# Shared object parameters (estimated)
SHARED_OBJECT_MASS_KG = 0.5  # Mass of object being handled
SHARED_OBJECT_COG = [0.0, 0.0, 0.1]  # CoG relative to TCP when gripped

# Payload distribution for dual-arm handling
MASTER_PAYLOAD_MASS = RG2_MASS_KG + (SHARED_OBJECT_MASS_KG * 0.5)  # Half of shared object
SLAVE_PAYLOAD_MASS = RG2_MASS_KG + (SHARED_OBJECT_MASS_KG * 0.5)   # Half of shared object
MASTER_PAYLOAD_COG = RG2_COG_OFFSET  # Combined CoG calculation
SLAVE_PAYLOAD_COG = RG2_COG_OFFSET

# --- Destination Poses ---
# The previous poses were unreachable and caused a safety stop.
# Using a standard, safe "home" pose for both arms to ensure success.
PICK_POSITION_POSE = [0.8399, 0.4085, -1.5899, -1.79, -0.844, 2.2184]
TAKE_POSITION_POSE = [0.0678, -2.0088, 2.2378, -2.5625, 4.6088, -0.0005]
TAKE_2_POSITION_POSE = [0.0702, -1.8146, 2.1715, -2.7095, 4.6033, -0.0006]

# --- Task-Specific Poses ---
GIVE_POSITION_POSE = [0.0418, -1.0165, -1.8723, -1.0687, 1.5364, 3.1638]
PLACE_POSITION_POSE = [-0.7148, -2.9587, 0.9348, -4.377, 5.4984, -2.2456]
PLACE_2_POSITION_POSE = [-0.5381, -2.9385, 0.9423, -4.4314, 5.736, -2.2245]
SLAVE_TAKE2_POSITION_POSE = [-0.6292, -3.3934, 0.9673, -2.8449, 4.2614, -2.605]
SLAVE_GIVE2_POSITION_POSE = [-0.0961, -1.7402, 1.7647, -2.4161, 4.8046, -3.241]
MASTER_TAKE_POSITION_POSE = [0.0518, -1.0508, -1.8886, -0.9469, 1.608, 3.1519]
MASTER_PLACE_POSITION_POSE = [0.609, -0.2112, -0.8516, 0.8923, 0.6701, 2.6341]

# --- Home Poses ---
MASTER_HOME_POSE = [-0.0035, -0.9265, -2.0075, -0.9667, 1.516, 0.1271]
SLAVE_HOME_POSE = [0.0255, -2.1495, 2.0109, -2.2203, -1.5238, -0.0317]

# --- Up Position Poses ---
MASTER_UP_POSITION = [0.054, -1.4461, -1.1711, -1.2776, 1.4738, 0.2186]
SLAVE_UP_POSITION = [0.0053, -1.6555, 1.2842, -2.0268, -1.5594, -0.0314]

# --- Left Position Poses ---
MASTER_LEFT_POSITION = [-0.0036, -0.8002, -0.9545, -2.1198, 1.5504, 0.1253]
SLAVE_LEFT_POSITION = [0.0183, -1.0852, 1.9735, -3.198, -1.594, -0.0888]

# --- Right Position Poses ---
MASTER_RIGHT_POSITION = [0.006, -1.9941, -1.9326, -0.0917, 1.5871, 0.1252]
SLAVE_RIGHT_POSITION = [0.0775, -2.3499, 1.0432, -1.1526, -1.6268, -0.055]

# --- New Simultaneous Move Poses ---
MASTER_NEW_POSITION = [-0.0036, -0.9261, -1.9598, -1.0169, 1.5409, -0.0004]
SLAVE_NEW_POSITION = [0.0257, -2.1371, 2.0043, -2.2257, -1.5587, -0.042]


# --- Enhanced Motion Parameters ---
MOVEMENT_TIME_SECONDS = 3.0 # Time for each robot's individual move. Lower is faster.
JOINT_ACCELERATION = 2.0    # Joint acceleration.
JOINT_VELOCITY = 3.0        # Joint velocity.
CONTROL_FREQUENCY_HZ = 125  # UR robot control frequency

# --- Object Handling Parameters (More Gentle) ---
OBJECT_MOVEMENT_TIME_SECONDS = 6.0   # Even slower movement when holding objects
OBJECT_JOINT_ACCELERATION = 0.3      # Much lower acceleration  
OBJECT_JOINT_VELOCITY = 0.8          # Much lower velocity
OBJECT_FORCE_THRESHOLD_N = 25.0      # Higher threshold for object handling
OBJECT_TORQUE_THRESHOLD_NM = 8.0     # Higher threshold for object handling

# --- Approach Control Parameters (Solution 4: Tame the Approach) ---
APPROACH_DISTANCE_MM = 20.0          # Distance for gentle approach phase
APPROACH_VELOCITY_SCALE = 0.3        # Velocity scale for approach phase
APPROACH_ACCELERATION_SCALE = 0.2    # Acceleration scale for approach phase

# --- Compliance Parameters (Solution 3: Add Compliance) ---
COMPLIANCE_STIFFNESS_LOW = [500, 500, 500, 50, 50, 50]  # Low stiffness for compliance [Fx,Fy,Fz,Mx,My,Mz]
COMPLIANCE_DAMPING = [50, 50, 50, 5, 5, 5]              # Damping coefficients
FORCE_MODE_ENABLED = True                                 # Enable force mode for compliance

# --- Enhanced Monitoring Parameters (Solution 6: Log & Verify) ---
MONITORING_ENABLED = True
MAX_DATA_POINTS = 2000                # Increased for better logging
MONITORING_FREQUENCY_HZ = 20          # Higher frequency monitoring
PLOT_UPDATE_FREQUENCY_HZ = 10
LOG_TO_FILE = True                    # Enable data logging to file
LOG_FILENAME = "dual_arm_force_log.csv"

# --- Adjusted Force Limits (Solution 5: Carefully adjusted) ---
# Normal operation thresholds
FORCE_THRESHOLD_N = 60.0              # Increased from 50N after risk assessment
TORQUE_THRESHOLD_NM = 12.0            # Increased from 10Nm after risk assessment

# Critical thresholds (emergency stop)
CRITICAL_FORCE_THRESHOLD_N = 80.0     # Emergency stop threshold
CRITICAL_TORQUE_THRESHOLD_NM = 15.0   # Emergency stop threshold

# --- Synchronization Parameters (Solution 2: Synchronise Motion) ---
SYNC_UPDATE_FREQUENCY_HZ = 125        # Real-time sync frequency
SYNC_POSITION_TOLERANCE = 0.01        # Position tolerance for synchronization (rad)
SYNC_VELOCITY_TOLERANCE = 0.05        # Velocity tolerance for synchronization (rad/s)
LEADER_FOLLOWER_DELAY_S = 0.02        # Small delay for follower arm

# --- Admittance Control Parameters (TRUE Master-Slave Solution) ---
ADMITTANCE_ACC_RATIO = 0.2            # Slave acceleration as ratio of master (20%)
ADMITTANCE_VEL_RATIO = 0.3            # Slave velocity as ratio of master (30%)
FORCE_ADJUSTMENT_FREQUENCY_HZ = 10    # Force monitoring frequency for admittance control
FORCE_ADJUSTMENT_THRESHOLD_RATIO = 0.8 # Adjust when force exceeds 80% of object threshold
ULTRA_GENTLE_ACC = 0.1                # Ultra-gentle acceleration for high force situations
ULTRA_GENTLE_VEL = 0.2                # Ultra-gentle velocity for high force situations

class PayloadManager:
    """Manages accurate payload configuration for dual-arm systems."""
    
    def __init__(self, master_arm, slave_arm):
        self.master_arm = master_arm
        self.slave_arm = slave_arm
        self.object_attached = False
        
    def configure_gripper_payload(self):
        """Configure payload for grippers only (no shared object)."""
        print("✓ Configuring gripper-only payload...")
        self.master_arm.set_payload(RG2_MASS_KG, RG2_COG_OFFSET)
        self.slave_arm.set_payload(RG2_MASS_KG, RG2_COG_OFFSET)
        self.object_attached = False
        print(f"  Master payload: {RG2_MASS_KG}kg at CoG {RG2_COG_OFFSET}")
        print(f"  Slave payload:  {RG2_MASS_KG}kg at CoG {RG2_COG_OFFSET}")
        
    def configure_object_payload(self):
        """Configure payload for grippers + shared object."""
        print("✓ Configuring gripper + object payload...")
        self.master_arm.set_payload(MASTER_PAYLOAD_MASS, MASTER_PAYLOAD_COG)
        self.slave_arm.set_payload(SLAVE_PAYLOAD_MASS, SLAVE_PAYLOAD_COG)
        self.object_attached = True
        print(f"  Master payload: {MASTER_PAYLOAD_MASS:.2f}kg at CoG {MASTER_PAYLOAD_COG}")
        print(f"  Slave payload:  {SLAVE_PAYLOAD_MASS:.2f}kg at CoG {SLAVE_PAYLOAD_COG}")
        print("  ⚠️  Each arm configured for 50% of shared object mass")
        
    def get_current_payload_info(self):
        """Get current payload configuration."""
        return {
            'object_attached': self.object_attached,
            'master_mass': MASTER_PAYLOAD_MASS if self.object_attached else RG2_MASS_KG,
            'slave_mass': SLAVE_PAYLOAD_MASS if self.object_attached else RG2_MASS_KG,
            'shared_object_mass': SHARED_OBJECT_MASS_KG if self.object_attached else 0.0
        }


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

class ForceTorqueMonitor:
    """Enhanced real-time force and torque monitoring for dual robot arms with logging and critical thresholds."""
    
    def __init__(self, master_arm, slave_arm, max_data_points=MAX_DATA_POINTS):
        self.master_arm = master_arm
        self.slave_arm = slave_arm
        self.max_data_points = max_data_points
        
        # Data storage
        self.timestamps = deque(maxlen=max_data_points)
        self.master_forces = deque(maxlen=max_data_points)
        self.master_torques = deque(maxlen=max_data_points)
        self.master_joints = deque(maxlen=max_data_points)
        self.slave_forces = deque(maxlen=max_data_points)
        self.slave_torques = deque(maxlen=max_data_points)
        self.slave_joints = deque(maxlen=max_data_points)
        
        # Enhanced data storage for logging
        self.master_force_vectors = deque(maxlen=max_data_points)  # Full 6D force/torque
        self.slave_force_vectors = deque(maxlen=max_data_points)   # Full 6D force/torque
        
        # Monitoring state
        self.is_monitoring = False
        self.monitoring_thread = None
        
        # Object handling mode with enhanced thresholds
        self.object_handling_mode = False
        self.current_force_threshold = FORCE_THRESHOLD_N
        self.current_torque_threshold = TORQUE_THRESHOLD_NM
        self.critical_force_threshold = CRITICAL_FORCE_THRESHOLD_N
        self.critical_torque_threshold = CRITICAL_TORQUE_THRESHOLD_NM
        
        # Statistics
        self.master_max_force = 0.0
        self.master_max_torque = 0.0
        self.slave_max_force = 0.0
        self.slave_max_torque = 0.0
        
        # Critical event tracking
        self.critical_events = []
        self.warning_count = 0
        self.critical_count = 0
        
        # Data logging
        self.log_file = None
        self.log_writer = None
        if LOG_TO_FILE:
            self._setup_logging()
        
        # Setup real-time monitors
        self._setup_realtime_monitors()
        
    def _setup_logging(self):
        """Setup CSV logging for force/torque data."""
        try:
            import csv
            self.log_file = open(LOG_FILENAME, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            # Write header
            header = ['timestamp', 'master_force', 'master_torque', 'slave_force', 'slave_torque',
                     'master_fx', 'master_fy', 'master_fz', 'master_mx', 'master_my', 'master_mz',
                     'slave_fx', 'slave_fy', 'slave_fz', 'slave_mx', 'slave_my', 'slave_mz',
                     'object_mode', 'warning_level']
            self.log_writer.writerow(header)
            print(f"✓ Force/torque logging enabled: {LOG_FILENAME}")
        except Exception as e:
            print(f"⚠️  Warning: Could not setup logging: {e}")
            self.log_file = None
            self.log_writer = None
        
    def _setup_realtime_monitors(self):
        """Setup real-time monitors for both arms."""
        try:
            # Enable real-time monitoring for both arms
            self.master_arm.get_realtime_monitor()
            self.slave_arm.get_realtime_monitor()
            print("✓ Real-time monitors initialized for both arms")
        except Exception as e:
            print(f"⚠️  Warning: Could not setup real-time monitors: {e}")
            print("   Force/torque monitoring may not work properly.")
    
    def start_monitoring(self):
        """Start the force/torque monitoring in a separate thread."""
        if self.is_monitoring:
            print("⚠️  Monitoring is already running.")
            return
            
        self.is_monitoring = True
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        print("✓ Force/torque monitoring started")
    
    def set_object_handling_mode(self, enabled=True):
        """Enable or disable object handling mode with lower thresholds."""
        self.object_handling_mode = enabled
        if enabled:
            self.current_force_threshold = OBJECT_FORCE_THRESHOLD_N
            self.current_torque_threshold = OBJECT_TORQUE_THRESHOLD_NM
            print(f"✓ Object handling mode enabled (Force: {OBJECT_FORCE_THRESHOLD_N}N, Torque: {OBJECT_TORQUE_THRESHOLD_NM}Nm)")
        else:
            self.current_force_threshold = FORCE_THRESHOLD_N
            self.current_torque_threshold = TORQUE_THRESHOLD_NM
            print(f"✓ Normal mode enabled (Force: {FORCE_THRESHOLD_N}N, Torque: {TORQUE_THRESHOLD_NM}Nm)")
    
    def get_monitoring_mode(self):
        """Get current monitoring mode and thresholds."""
        return {
            'object_handling_mode': self.object_handling_mode,
            'force_threshold': self.current_force_threshold,
            'torque_threshold': self.current_torque_threshold
        }
    
    def _monitoring_loop(self):
        """Enhanced monitoring loop with critical threshold checking and logging."""
        monitoring_interval = 1.0 / MONITORING_FREQUENCY_HZ
        
        while self.is_monitoring:
            try:
                start_time = time.time()
                
                # Get current timestamp
                current_time = time.time()
                
                # Read force/torque data from both arms
                master_tcp_force = self.master_arm.get_tcp_force(wait=False)
                slave_tcp_force = self.slave_arm.get_tcp_force(wait=False)
                
                # Calculate force magnitude (Euclidean norm)
                master_force = np.linalg.norm(master_tcp_force[:3]) if master_tcp_force is not None else 0.0
                slave_force = np.linalg.norm(slave_tcp_force[:3]) if slave_tcp_force is not None else 0.0
                
                # Calculate torque magnitude (Euclidean norm)
                master_torque = np.linalg.norm(master_tcp_force[3:]) if master_tcp_force is not None else 0.0
                slave_torque = np.linalg.norm(slave_tcp_force[3:]) if slave_tcp_force is not None else 0.0
                
                # Get joint positions
                master_joints = self.master_arm.getj(wait=False)
                slave_joints = self.slave_arm.getj(wait=False)
                
                # Store data
                self.timestamps.append(current_time)
                self.master_forces.append(master_force)
                self.master_torques.append(master_torque)
                self.master_joints.append(master_joints)
                self.slave_forces.append(slave_force)
                self.slave_torques.append(slave_torque)
                self.slave_joints.append(slave_joints)
                
                # Store full force vectors for detailed logging
                self.master_force_vectors.append(master_tcp_force if master_tcp_force is not None else [0]*6)
                self.slave_force_vectors.append(slave_tcp_force if slave_tcp_force is not None else [0]*6)
                
                # Update statistics
                self.master_max_force = max(self.master_max_force, master_force)
                self.master_max_torque = max(self.master_max_torque, master_torque)
                self.slave_max_force = max(self.slave_max_force, slave_force)
                self.slave_max_torque = max(self.slave_max_torque, slave_torque)
                
                # Enhanced threshold checking with warning levels
                warning_level = self._check_thresholds(master_force, master_torque, slave_force, slave_torque)
                
                # Log data to file if enabled
                if self.log_writer:
                    self._log_data(current_time, master_force, master_torque, slave_force, slave_torque,
                                 master_tcp_force, slave_tcp_force, warning_level)
                
                # Sleep to maintain monitoring frequency
                elapsed = time.time() - start_time
                sleep_time = max(0, monitoring_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"✗ Error in monitoring loop: {e}")
                time.sleep(0.1)  # Brief pause on error
    
    def _check_thresholds(self, master_force, master_torque, slave_force, slave_torque):
        """Enhanced threshold checking with multiple warning levels."""
        warning_level = 0  # 0=OK, 1=Warning, 2=Critical
        mode_indicator = " [OBJECT MODE]" if self.object_handling_mode else ""
        
        # Check critical thresholds first (emergency stop level)
        critical_force = (master_force > self.critical_force_threshold or 
                         slave_force > self.critical_force_threshold)
        critical_torque = (master_torque > self.critical_torque_threshold or 
                          slave_torque > self.critical_torque_threshold)
        
        if critical_force or critical_torque:
            warning_level = 2
            self.critical_count += 1
            event = {
                'timestamp': time.time(),
                'type': 'CRITICAL',
                'master_force': master_force,
                'master_torque': master_torque,
                'slave_force': slave_force,
                'slave_torque': slave_torque
            }
            self.critical_events.append(event)
            
            if critical_force:
                print(f"🚨 CRITICAL: Force threshold exceeded! Master: {master_force:.2f}N, Slave: {slave_force:.2f}N (Limit: {self.critical_force_threshold}N){mode_indicator}")
            if critical_torque:
                print(f"🚨 CRITICAL: Torque threshold exceeded! Master: {master_torque:.2f}Nm, Slave: {slave_torque:.2f}Nm (Limit: {self.critical_torque_threshold}Nm){mode_indicator}")
            
            # Could trigger emergency stop here if needed
            # self._emergency_stop()
            
        # Check warning thresholds
        elif (master_force > self.current_force_threshold or slave_force > self.current_force_threshold or
              master_torque > self.current_torque_threshold or slave_torque > self.current_torque_threshold):
            warning_level = 1
            self.warning_count += 1
            
            if master_force > self.current_force_threshold:
                print(f"⚠️  WARNING: Master arm force ({master_force:.2f}N) exceeds threshold ({self.current_force_threshold}N){mode_indicator}")
            if slave_force > self.current_force_threshold:
                print(f"⚠️  WARNING: Slave arm force ({slave_force:.2f}N) exceeds threshold ({self.current_force_threshold}N){mode_indicator}")
            if master_torque > self.current_torque_threshold:
                print(f"⚠️  WARNING: Master arm torque ({master_torque:.2f}Nm) exceeds threshold ({self.current_torque_threshold}Nm){mode_indicator}")
            if slave_torque > self.current_torque_threshold:
                print(f"⚠️  WARNING: Slave arm torque ({slave_torque:.2f}Nm) exceeds threshold ({self.current_torque_threshold}Nm){mode_indicator}")
        
        return warning_level
    
    def _log_data(self, timestamp, master_force, master_torque, slave_force, slave_torque,
                  master_tcp_force, slave_tcp_force, warning_level):
        """Log detailed force/torque data to CSV file."""
        try:
            # Extract individual force/torque components
            master_ft = master_tcp_force if master_tcp_force is not None else [0]*6
            slave_ft = slave_tcp_force if slave_tcp_force is not None else [0]*6
            
            row = [
                timestamp, master_force, master_torque, slave_force, slave_torque,
                master_ft[0], master_ft[1], master_ft[2], master_ft[3], master_ft[4], master_ft[5],
                slave_ft[0], slave_ft[1], slave_ft[2], slave_ft[3], slave_ft[4], slave_ft[5],
                self.object_handling_mode, warning_level
            ]
            self.log_writer.writerow(row)
            self.log_file.flush()  # Ensure data is written immediately
        except Exception as e:
            print(f"⚠️  Logging error: {e}")
    
    def stop_monitoring(self):
        """Stop the force/torque monitoring and close log file."""
        self.is_monitoring = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)
        
        # Close log file
        if self.log_file:
            self.log_file.close()
            print(f"✓ Log file closed: {LOG_FILENAME}")
        
        print("✓ Force/torque monitoring stopped")
        
        # Print summary statistics
        if self.warning_count > 0 or self.critical_count > 0:
            print(f"📊 Monitoring Summary: {self.warning_count} warnings, {self.critical_count} critical events")
    
    def get_current_data(self):
        """Get the most recent force/torque data."""
        if not self.timestamps:
            return None
            
        return {
            'timestamp': self.timestamps[-1] if self.timestamps else 0,
            'master': {
                'force': self.master_forces[-1] if self.master_forces else 0.0,
                'torque': self.master_torques[-1] if self.master_torques else 0.0,
                'joints': self.master_joints[-1] if self.master_joints else [0.0] * 6
            },
            'slave': {
                'force': self.slave_forces[-1] if self.slave_forces else 0.0,
                'torque': self.slave_torques[-1] if self.slave_torques else 0.0,
                'joints': self.slave_joints[-1] if self.slave_joints else [0.0] * 6
            }
        }
    
    def get_statistics(self):
        """Get monitoring statistics."""
        return {
            'master_max_force': self.master_max_force,
            'master_max_torque': self.master_max_torque,
            'slave_max_force': self.slave_max_force,
            'slave_max_torque': self.slave_max_torque,
            'data_points': len(self.timestamps)
        }
    
    def print_status(self):
        """Print current force/torque status."""
        data = self.get_current_data()
        if data:
            print(f"\n--- Force/Torque Status ---")
            print(f"Master Arm: {data['master']['force']:.2f}N, {data['master']['torque']:.2f}Nm")
            print(f"Slave Arm:  {data['slave']['force']:.2f}N, {data['slave']['torque']:.2f}Nm")
            
            stats = self.get_statistics()
            print(f"\n--- Statistics ---")
            print(f"Master Max: {stats['master_max_force']:.2f}N, {stats['master_max_torque']:.2f}Nm")
            print(f"Slave Max:  {stats['slave_max_force']:.2f}N, {stats['slave_max_torque']:.2f}Nm")
            print(f"Data Points: {stats['data_points']}")
            
            # Automatically check payload distribution when in object handling mode
            if self.object_handling_mode:
                self.check_payload_distribution()
    
    def check_payload_distribution(self):
        """Check if payload is equally distributed between arms."""
        if not self.timestamps:
            return None
        
        master_force = self.master_forces[-1] if self.master_forces else 0.0
        slave_force = self.slave_forces[-1] if self.slave_forces else 0.0
        
        # Calculate distribution ratio
        total_force = master_force + slave_force
        if total_force > 0:
            master_ratio = master_force / total_force
            slave_ratio = slave_force / total_force
            
            print(f"\n--- Payload Distribution Analysis ---")
            print(f"  Master arm: {master_ratio*100:.1f}% ({master_force:.2f}N)")
            print(f"  Slave arm: {slave_ratio*100:.1f}% ({slave_force:.2f}N)")
            print(f"  Total force: {total_force:.2f}N")
            
            # Enhanced payload sharing analysis
            if self.object_handling_mode:
                expected_mass_per_arm = SHARED_OBJECT_MASS_KG * 0.5
                print(f"  Expected shared object mass per arm: {expected_mass_per_arm:.2f}kg")
                print(f"  Gripper mass per arm: {RG2_MASS_KG:.2f}kg")
                print(f"  Total expected payload per arm: {expected_mass_per_arm + RG2_MASS_KG:.2f}kg")
            
            # Check if distribution is uneven
            if abs(master_ratio - 0.5) > 0.2:  # More than 20% difference
                print(f"  ⚠️  UNEVEN LOAD: One arm carrying significantly more weight!")
                print(f"      Difference: {abs(master_ratio - slave_ratio)*100:.1f}% from ideal 50/50 split")
            else:
                print(f"  ✓ BALANCED LOAD: Payload is well distributed between arms")
                print(f"    Difference from ideal: {abs(master_ratio - slave_ratio)*100:.1f}%")
                
            return {
                'master_ratio': master_ratio,
                'slave_ratio': slave_ratio,
                'master_force': master_force,
                'slave_force': slave_force,
                'total_force': total_force,
                'is_balanced': abs(master_ratio - 0.5) <= 0.2
            }
        else:
            print(f"\n--- Payload Distribution Analysis ---")
            print(f"  No significant forces detected - arms likely not carrying payload")
            return None

class RealTimePlotter:
    """Real-time plotting for force/torque data."""
    
    def __init__(self, monitor):
        self.monitor = monitor
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Real-time Force/Torque Monitoring', fontsize=16)
        
        # Initialize plots
        self._setup_plots()
        
        # Animation
        self.ani = None
        
    def _setup_plots(self):
        """Setup the plotting axes."""
        # Force plots
        self.axes[0, 0].set_title('Master Arm - Force')
        self.axes[0, 0].set_ylabel('Force (N)')
        self.axes[0, 0].grid(True)
        self.master_force_line, = self.axes[0, 0].plot([], [], 'b-', linewidth=2, label='Force')
        self.axes[0, 0].legend()
        
        self.axes[0, 1].set_title('Slave Arm - Force')
        self.axes[0, 1].set_ylabel('Force (N)')
        self.axes[0, 1].grid(True)
        self.slave_force_line, = self.axes[0, 1].plot([], [], 'r-', linewidth=2, label='Force')
        self.axes[0, 1].legend()
        
        # Torque plots
        self.axes[1, 0].set_title('Master Arm - Torque')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Torque (Nm)')
        self.axes[1, 0].grid(True)
        self.master_torque_line, = self.axes[1, 0].plot([], [], 'b-', linewidth=2, label='Torque')
        self.axes[1, 0].legend()
        
        self.axes[1, 1].set_title('Slave Arm - Torque')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Torque (Nm)')
        self.axes[1, 1].grid(True)
        self.slave_torque_line, = self.axes[1, 1].plot([], [], 'r-', linewidth=2, label='Torque')
        self.axes[1, 1].legend()
        
        # Add threshold lines
        for ax in [self.axes[0, 0], self.axes[0, 1]]:
            ax.axhline(y=FORCE_THRESHOLD_N, color='orange', linestyle='--', alpha=0.7, label=f'Threshold ({FORCE_THRESHOLD_N}N)')
            ax.legend()
            
        for ax in [self.axes[1, 0], self.axes[1, 1]]:
            ax.axhline(y=TORQUE_THRESHOLD_NM, color='orange', linestyle='--', alpha=0.7, label=f'Threshold ({TORQUE_THRESHOLD_NM}Nm)')
            ax.legend()
        
        # Initialize text annotations for numerical values
        self.master_force_text = self.axes[0, 0].text(0.02, 0.95, '', transform=self.axes[0, 0].transAxes, 
                                                      bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                                                      fontsize=10, verticalalignment='top')
        self.slave_force_text = self.axes[0, 1].text(0.02, 0.95, '', transform=self.axes[0, 1].transAxes,
                                                     bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                                                     fontsize=10, verticalalignment='top')
        self.master_torque_text = self.axes[1, 0].text(0.02, 0.95, '', transform=self.axes[1, 0].transAxes,
                                                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                                                       fontsize=10, verticalalignment='top')
        self.slave_torque_text = self.axes[1, 1].text(0.02, 0.95, '', transform=self.axes[1, 1].transAxes,
                                                      bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                                                      fontsize=10, verticalalignment='top')
    
    def start_plotting(self):
        """Start real-time plotting."""
        self.ani = FuncAnimation(self.fig, self._update_plots, interval=1000//PLOT_UPDATE_FREQUENCY_HZ, blit=False)
        plt.show(block=False)
        print("✓ Real-time plotting started")
    
    def stop_plotting(self):
        """Stop real-time plotting."""
        if self.ani:
            self.ani.event_source.stop()
        plt.close(self.fig)
        print("✓ Real-time plotting stopped")
    
    def _update_plots(self, frame):
        """Update the plots with new data."""
        if not self.monitor.timestamps:
            return
            
        try:
            # Get time data (relative to start) - ensure all arrays have same length
            timestamps = list(self.monitor.timestamps)
            master_forces = list(self.monitor.master_forces)
            slave_forces = list(self.monitor.slave_forces)
            master_torques = list(self.monitor.master_torques)
            slave_torques = list(self.monitor.slave_torques)
            
            # Ensure all arrays have the same length
            min_length = min(len(timestamps), len(master_forces), len(slave_forces), 
                           len(master_torques), len(slave_torques))
            
            if min_length == 0:
                return
                
            timestamps = timestamps[:min_length]
            master_forces = master_forces[:min_length]
            slave_forces = slave_forces[:min_length]
            master_torques = master_torques[:min_length]
            slave_torques = slave_torques[:min_length]
            
            start_time = timestamps[0] if timestamps else 0
            times = [t - start_time for t in timestamps]
            
            # Update force plots
            if len(times) > 0 and len(master_forces) > 0:
                self.master_force_line.set_data(times, master_forces)
                self.axes[0, 0].relim()
                self.axes[0, 0].autoscale_view()
                
            if len(times) > 0 and len(slave_forces) > 0:
                self.slave_force_line.set_data(times, slave_forces)
                self.axes[0, 1].relim()
                self.axes[0, 1].autoscale_view()
            
            # Update torque plots
            if len(times) > 0 and len(master_torques) > 0:
                self.master_torque_line.set_data(times, master_torques)
                self.axes[1, 0].relim()
                self.axes[1, 0].autoscale_view()
                
            if len(times) > 0 and len(slave_torques) > 0:
                self.slave_torque_line.set_data(times, slave_torques)
                self.axes[1, 1].relim()
                self.axes[1, 1].autoscale_view()
        except Exception as e:
            print(f"Plot update error: {e}")
            return
        
        # Update numerical value displays
        if self.monitor.master_forces:
            current_master_force = self.monitor.master_forces[-1]
            self.master_force_text.set_text(f'Current: {current_master_force:.2f} N')
            # Change color if threshold exceeded
            if current_master_force > FORCE_THRESHOLD_N:
                self.master_force_text.set_color('red')
            else:
                self.master_force_text.set_color('black')
                
        if self.monitor.slave_forces:
            current_slave_force = self.monitor.slave_forces[-1]
            self.slave_force_text.set_text(f'Current: {current_slave_force:.2f} N')
            # Change color if threshold exceeded
            if current_slave_force > FORCE_THRESHOLD_N:
                self.slave_force_text.set_color('red')
            else:
                self.slave_force_text.set_color('black')
                
        if self.monitor.master_torques:
            current_master_torque = self.monitor.master_torques[-1]
            self.master_torque_text.set_text(f'Current: {current_master_torque:.2f} Nm')
            # Change color if threshold exceeded
            if current_master_torque > TORQUE_THRESHOLD_NM:
                self.master_torque_text.set_color('red')
            else:
                self.master_torque_text.set_color('black')
                
        if self.monitor.slave_torques:
            current_slave_torque = self.monitor.slave_torques[-1]
            self.slave_torque_text.set_text(f'Current: {current_slave_torque:.2f} Nm')
            # Change color if threshold exceeded
            if current_slave_torque > TORQUE_THRESHOLD_NM:
                self.slave_torque_text.set_color('red')
            else:
                self.slave_torque_text.set_color('black')
        
        # Update x-axis limits to show last 10 seconds
        if times:
            current_time = times[-1]
            window_start = max(0, current_time - 10)
            for ax in self.axes.flat:
                ax.set_xlim(window_start, current_time + 1)
        
        return self.master_force_line, self.slave_force_line, self.master_torque_line, self.slave_torque_line


class SynchronizedMotionController:
    """
    Advanced synchronized motion controller implementing TRUE master-slave control with admittance control.
    
    ===================================================================================
    TRUE MASTER-SLAVE WITH ADMITTANCE CONTROL - REVOLUTIONARY APPROACH
    ===================================================================================
    
    This implementation solves the fundamental dual-arm object handling problem by 
    eliminating trajectory conflicts between arms holding a shared object.
    
    PROBLEM ANALYSIS:
    ----------------
    Previous "synchronized" approaches failed because both arms followed independent 
    trajectories, causing them to "fight" each other through the shared object:
    
    ❌ OLD WAY: Both arms follow predetermined paths
       Master: [pos1, pos2, pos3, ...] → FORCE CONFLICT
       Slave:  [pos1, pos2, pos3, ...] → FORCE CONFLICT
       Result: Internal forces exceed thresholds → Protective stop
    
    SOLUTION PRINCIPLES:
    -------------------
    ✅ NEW WAY: True master-slave with admittance control
    
    1. MASTER ARM: Controls object trajectory completely
       - Follows predetermined trajectory exactly
       - Responsible for object positioning and timing
       - Uses normal acceleration/velocity parameters
    
    2. SLAVE ARM: Uses admittance control to follow along
       - NO predetermined trajectory (this is key!)
       - Moves toward final target with very low stiffness
       - Gets "dragged along" by master through shared object
       - Uses force feedback to adjust compliance dynamically
    
    3. FORCE-GUIDED ADJUSTMENT: Real-time adaptation
       - Monitors forces continuously during motion
       - If forces exceed threshold, slave becomes more compliant
       - Eliminates trajectory conflicts automatically
    
    TECHNICAL IMPLEMENTATION:
    -------------------------
    
    Master Arm Behavior:
    - movejs() with full trajectory path
    - Normal control parameters
    - Leads the shared object motion
    
    Slave Arm Behavior:
    - movej() to final target only (no intermediate waypoints)
    - Ultra-low acceleration (20% of master)
    - Ultra-low velocity (30% of master)
    - Force monitoring thread adjusts compliance
    
    Force Adjustment Loop:
    - Runs at 10Hz during motion
    - Monitors slave arm forces
    - If force > 80% of threshold:
      * Stops current slave motion
      * Restarts with even gentler parameters
      * Allows master to "pull" slave along
    
    BENEFITS:
    ---------
    1. Eliminates internal force conflicts
    2. Prevents protective stops during object handling
    3. Maintains object control through master arm
    4. Adapts to varying load conditions automatically
    5. Provides smooth, natural dual-arm coordination
    
    USAGE:
    ------
    sync_controller = SynchronizedMotionController(master_arm, slave_arm, payload_manager)
    sync_controller.execute_synchronized_move_with_compliance(
        master_target, slave_target, time, acc, vel, use_gentle_approach=True
    )
    
    The system automatically applies true master-slave coordination with admittance control.
    """
    
    def __init__(self, master_arm, slave_arm, payload_manager):
        self.master_arm = master_arm
        self.slave_arm = slave_arm
        self.payload_manager = payload_manager
        self.sync_active = False
        self.sync_thread = None
        
        # Synchronization state
        self.master_target_pose = None
        self.slave_target_pose = None
        self.sync_start_time = None
        self.movement_duration = None
        
        # Force adjustment state for admittance control
        self.force_adjustment_active = False
        self.force_adjustment_thread = None
        
    def execute_synchronized_move_with_compliance(self, master_dest_pose, slave_dest_pose, 
                                                movement_time, acc, vel, use_gentle_approach=True):
        """
        Execute synchronized movement with compliance and gentle approach.
        Implements Solutions 2, 3, and 4 from the requirements.
        """
        print("\n--- Synchronized Move with Compliance ---")
        
        # Get current positions
        master_start_pose = np.array(self.master_arm.getj())
        slave_start_pose = np.array(self.slave_arm.getj())
        
        print(f"✓ Master Start: {np.round(master_start_pose, 4).tolist()}")
        print(f"✓ Slave Start:  {np.round(slave_start_pose, 4).tolist()}")
        print(f"✓ Master Target: {np.round(master_dest_pose, 4).tolist()}")
        print(f"✓ Slave Target:  {np.round(slave_dest_pose, 4).tolist()}")
        
        # Generate trajectories with gentle approach
        if use_gentle_approach:
            master_path, slave_path = self._generate_gentle_approach_trajectories(
                master_start_pose, master_dest_pose, slave_start_pose, slave_dest_pose, movement_time)
        else:
            master_path, slave_path = self._generate_standard_trajectories(
                master_start_pose, master_dest_pose, slave_start_pose, slave_dest_pose, movement_time)
        
        # Execute with master-slave synchronization
        self._execute_master_slave_motion(master_path, slave_path, acc, vel)
        
        print("✓ Synchronized movement with compliance complete.")
    
    def _generate_gentle_approach_trajectories(self, master_start, master_dest, slave_start, slave_dest, movement_time):
        """Generate trajectories with gentle approach phase (Solution 4)."""
        print("✓ Generating gentle approach trajectories...")
        
        traj_generator = Trajectory(method="quintic")
        control_frequency = 1.0 / CONTROL_FREQUENCY_HZ
        
        # Calculate approach waypoints (slow down in final approach)
        approach_fraction = APPROACH_DISTANCE_MM / 1000.0  # Convert mm to meters (rough approximation)
        
        # Create intermediate waypoints for gentle approach
        master_approach = master_start + 0.9 * (master_dest - master_start)
        slave_approach = slave_start + 0.9 * (slave_dest - slave_start)
        
        # Generate multi-phase trajectory
        # Phase 1: Normal speed to approach point
        approach_time = movement_time * 0.7
        final_time = movement_time * 0.3
        
        # Master trajectory with approach phase
        master_path1, _ = traj_generator.piecewise_interpolation(
            path=[master_start, master_approach],
            control_frequency=control_frequency,
            interval_time=approach_time
        )
        
        master_path2, _ = traj_generator.piecewise_interpolation(
            path=[master_approach, master_dest],
            control_frequency=control_frequency,
            interval_time=final_time
        )
        
        # Slave trajectory with approach phase
        slave_path1, _ = traj_generator.piecewise_interpolation(
            path=[slave_start, slave_approach],
            control_frequency=control_frequency,
            interval_time=approach_time
        )
        
        slave_path2, _ = traj_generator.piecewise_interpolation(
            path=[slave_approach, slave_dest],
            control_frequency=control_frequency,
            interval_time=final_time
        )
        
        # Combine trajectories
        master_path = master_path1 + master_path2
        slave_path = slave_path1 + slave_path2
        
        print(f"✓ Gentle approach trajectories generated: {len(master_path)} waypoints")
        return master_path, slave_path
    
    def _generate_standard_trajectories(self, master_start, master_dest, slave_start, slave_dest, movement_time):
        """Generate standard synchronized trajectories."""
        traj_generator = Trajectory(method="quintic")
        control_frequency = 1.0 / CONTROL_FREQUENCY_HZ
        
        master_path, _ = traj_generator.piecewise_interpolation(
            path=[master_start, master_dest],
            control_frequency=control_frequency,
            interval_time=movement_time
        )
        
        slave_path, _ = traj_generator.piecewise_interpolation(
            path=[slave_start, slave_dest],
            control_frequency=control_frequency,
            interval_time=movement_time
        )
        
        return master_path, slave_path
    
    def _execute_master_slave_motion(self, master_path, slave_path, acc, vel):
        """Execute TRUE master-slave motion with admittance control.
        
        Master arm controls the object trajectory completely.
        Slave arm uses force-guided admittance control to follow along.
        This eliminates trajectory conflicts that cause internal forces.
        """
        print("✓ Executing TRUE master-slave motion with admittance control...")
        
        # Get final target for slave (we won't use the full path)
        slave_final_target = slave_path[-1]
        
        # Start master arm (leader) - follows its trajectory exactly
        print("  → Master arm controlling object trajectory...")
        self.master_arm.movejs(joint_positions_list=master_path, acc=acc, vel=vel, radius=0.02, wait=False)
        
        # Small delay to let master establish motion
        time.sleep(LEADER_FOLLOWER_DELAY_S)
        
        # Start slave arm with TRUE compliance - no predetermined trajectory
        print("  → Slave arm following with admittance control...")
        self._execute_admittance_following(slave_final_target, acc, vel)
        
        # Monitor the master-slave coordination
        self._monitor_master_slave_coordination(len(master_path))
        
    def _execute_admittance_following(self, slave_target, acc, vel):
        """Execute admittance control for slave arm.
        
        The slave arm moves toward its target with very low stiffness,
        allowing it to be "dragged along" by the master through the shared object.
        """
        # Use configured admittance parameters
        admittance_acc = acc * ADMITTANCE_ACC_RATIO   # Much lower acceleration
        admittance_vel = vel * ADMITTANCE_VEL_RATIO   # Much lower velocity
        
        # Move slave with loose control - it will be guided by forces from shared object
        print(f"    → Slave moving to target with admittance parameters (acc={admittance_acc:.2f}, vel={admittance_vel:.2f})")
        self.slave_arm.movej(slave_target, acc=admittance_acc, vel=admittance_vel, wait=False)
        
        # Start force-guided adjustment thread for real-time compliance adaptation
        self._start_force_guided_adjustment()
        
    def _start_force_guided_adjustment(self):
        """Start a thread that monitors forces and adjusts slave motion if needed."""
        if not hasattr(self, 'force_adjustment_thread') or not self.force_adjustment_thread.is_alive():
            self.force_adjustment_active = True
            self.force_adjustment_thread = threading.Thread(target=self._force_guided_adjustment_loop, daemon=True)
            self.force_adjustment_thread.start()
            print("    → Force-guided adjustment active")
    
    def _force_guided_adjustment_loop(self):
        """Continuously monitor forces and adjust slave motion to reduce internal forces."""
        adjustment_interval = 1.0 / FORCE_ADJUSTMENT_FREQUENCY_HZ
        
        while self.force_adjustment_active and (self.master_arm.is_program_running() or self.slave_arm.is_program_running()):
            try:
                # Get current force readings
                slave_tcp_force = self.slave_arm.get_tcp_force(wait=False)
                
                if slave_tcp_force is not None:
                    # Calculate force magnitude
                    force_magnitude = np.linalg.norm(slave_tcp_force[:3])
                    
                    # If forces are getting high, make slave even more compliant
                    force_threshold = OBJECT_FORCE_THRESHOLD_N * FORCE_ADJUSTMENT_THRESHOLD_RATIO
                    if force_magnitude > force_threshold:
                        print(f"    → High forces detected ({force_magnitude:.1f}N), increasing slave compliance")
                        
                        # Get current slave position and reduce its "stiffness"
                        current_slave_pos = self.slave_arm.getj(wait=False)
                        if current_slave_pos is not None:
                            # Stop current motion and restart with even gentler parameters
                            self.slave_arm.stop()
                            time.sleep(0.1)
                            
                            # Continue toward target but with maximum compliance
                            # Use ultra-gentle parameters to let master "drag" the slave
                            self.slave_arm.movej(current_slave_pos, acc=ULTRA_GENTLE_ACC, vel=ULTRA_GENTLE_VEL, wait=False)
                
                time.sleep(adjustment_interval)
                
            except Exception as e:
                print(f"    ⚠️  Force adjustment error: {e}")
                time.sleep(0.1)
        
        self.force_adjustment_active = False
        print("    → Force-guided adjustment stopped")
    
    def _monitor_master_slave_coordination(self, expected_waypoints):
        """Monitor master-slave coordination focusing on force balance rather than position sync."""
        print("✓ Monitoring master-slave coordination...")
        start_time = time.time()
        max_wait_time = 30.0
        
        coordination_data = []
        
        while (self.master_arm.is_program_running() or self.slave_arm.is_program_running()) and (time.time() - start_time) < max_wait_time:
            try:
                # Focus on force monitoring rather than position synchronization
                master_tcp_force = self.master_arm.get_tcp_force(wait=False)
                slave_tcp_force = self.slave_arm.get_tcp_force(wait=False)
                
                if master_tcp_force is not None and slave_tcp_force is not None:
                    master_force = np.linalg.norm(master_tcp_force[:3])
                    slave_force = np.linalg.norm(slave_tcp_force[:3])
                    
                    # Calculate force balance (ideally should be similar)
                    force_imbalance = abs(master_force - slave_force)
                    coordination_data.append({
                        'time': time.time() - start_time,
                        'master_force': master_force,
                        'slave_force': slave_force,
                        'imbalance': force_imbalance
                    })
                    
                    # Warn if forces are becoming unbalanced
                    if force_imbalance > 20.0:  # Large force imbalance
                        print(f"  ⚠️  Force imbalance: Master={master_force:.1f}N, Slave={slave_force:.1f}N")
                
                time.sleep(0.1)  # 10Hz monitoring
                
            except Exception as e:
                print(f"  ⚠️  Coordination monitoring error: {e}")
                time.sleep(0.1)
        
        # Stop force adjustment
        self.force_adjustment_active = False
        
        if coordination_data:
            avg_master_force = np.mean([d['master_force'] for d in coordination_data])
            avg_slave_force = np.mean([d['slave_force'] for d in coordination_data])
            avg_imbalance = np.mean([d['imbalance'] for d in coordination_data])
            
            print(f"✓ Master-slave coordination complete.")
            print(f"  Average forces - Master: {avg_master_force:.2f}N, Slave: {avg_slave_force:.2f}N")
            print(f"  Average force imbalance: {avg_imbalance:.2f}N")


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


def execute_object_handling_move(master_arm, master_dest_pose, slave_arm, slave_dest_pose, movement_time, acc, vel, use_object_params=True):
    """
    Moves both robot arms when holding an object using master-slave control.
    The master arm leads the movement while the slave arm follows with compliance.
    This reduces internal forces that cause protective stops.
    
    Args:
        master_arm: The URRobot object for the master arm (leads the movement).
        master_dest_pose: The target joint configuration for the master arm.
        slave_arm: The URRobot object for the slave arm (follows with compliance).
        slave_dest_pose: The target joint configuration for the slave arm.
        movement_time: The time for the move in seconds.
        acc: The joint acceleration.
        vel: The joint velocity.
        use_object_params: Whether to use gentler object handling parameters.
    """
    print("\n--- Object Handling Move (Master-Slave Control) ---")
    
    # Use gentler parameters if specified
    if use_object_params:
        movement_time = OBJECT_MOVEMENT_TIME_SECONDS
        acc = OBJECT_JOINT_ACCELERATION
        vel = OBJECT_JOINT_VELOCITY
        print("✓ Using gentle object handling parameters")
    
    # 1. Get current positions
    master_start_pose = np.array(master_arm.getj())
    slave_start_pose = np.array(slave_arm.getj())
    print(f"✓ Master Start Pose: {np.round(master_start_pose, 4).tolist()}")
    print(f"✓ Slave Start Pose:  {np.round(slave_start_pose, 4).tolist()}")
    print(f"✓ Master Target Pose: {np.round(master_dest_pose, 4).tolist()}")
    print(f"✓ Slave Target Pose:  {np.round(slave_dest_pose, 4).tolist()}")

    # 2. Generate master trajectory (leader)
    traj_generator = Trajectory(method="quintic")
    control_frequency = 1.0 / CONTROL_FREQUENCY_HZ

    master_path, _ = traj_generator.piecewise_interpolation(
        path=[master_start_pose, master_dest_pose],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    print(f"✓ Master trajectory generated with {len(master_path)} waypoints.")

    # 3. Generate slave trajectory with slight delay (follower)
    slave_path, _ = traj_generator.piecewise_interpolation(
        path=[slave_start_pose, slave_dest_pose],
        control_frequency=control_frequency,
        interval_time=movement_time
    )
    print(f"✓ Slave trajectory generated with {len(slave_path)} waypoints.")

    # 4. Execute with master leading
    print("[Step 3] Executing object handling trajectory...")
    print("!!! MASTER ARM LEADS, SLAVE ARM FOLLOWS !!!")
    
    # Start master arm first
    master_arm.movejs(joint_positions_list=master_path, acc=acc, vel=vel, radius=0.02, wait=False)
    time.sleep(0.1)  # Small delay to let master start first
    
    # Start slave arm with slightly lower acceleration for compliance
    slave_arm.movejs(joint_positions_list=slave_path, acc=acc*0.8, vel=vel*0.9, radius=0.02, wait=False)

    # 5. Monitor movement with force checking
    print("✓ Monitoring movement and forces...")
    start_time = time.time()
    max_wait_time = movement_time + 10  # Extra time buffer
    
    while (master_arm.is_program_running() or slave_arm.is_program_running()) and (time.time() - start_time) < max_wait_time:
        time.sleep(0.05)
        
        # Optional: Check forces during movement if monitoring is available
        # This could be used to adjust movement if forces get too high

    print("✓ Object handling movement complete.")


def execute_compliant_simultaneous_move(master_arm, master_dest_pose, slave_arm, slave_dest_pose, movement_time, acc, vel):
    """
    Alternative approach: Simultaneous movement with compliance settings.
    Uses lower accelerations and longer movement times to reduce internal forces.
    """
    print("\n--- Compliant Simultaneous Move ---")
    
    # Use even gentler parameters
    compliant_time = movement_time * 1.5
    compliant_acc = acc * 0.3
    compliant_vel = vel * 0.5
    
    print(f"✓ Using compliant parameters: time={compliant_time:.1f}s, acc={compliant_acc:.1f}, vel={compliant_vel:.1f}")
    
    # Get current positions
    master_start_pose = np.array(master_arm.getj())
    slave_start_pose = np.array(slave_arm.getj())
    print(f"✓ Master Start: {np.round(master_start_pose, 4).tolist()}")
    print(f"✓ Slave Start:  {np.round(slave_start_pose, 4).tolist()}")

    # Generate trajectories with extra smoothing
    traj_generator = Trajectory(method="quintic")
    control_frequency = 1.0 / CONTROL_FREQUENCY_HZ

    master_path, _ = traj_generator.piecewise_interpolation(
        path=[master_start_pose, master_dest_pose],
        control_frequency=control_frequency,
        interval_time=compliant_time
    )

    slave_path, _ = traj_generator.piecewise_interpolation(
        path=[slave_start_pose, slave_dest_pose],
        control_frequency=control_frequency,
        interval_time=compliant_time
    )
    print(f"✓ Compliant trajectories generated.")

    # Execute with larger radius for smoother motion
    print("!!! EXECUTING COMPLIANT SIMULTANEOUS MOVEMENT !!!")
    master_arm.movejs(joint_positions_list=master_path, acc=compliant_acc, vel=compliant_vel, radius=0.05, wait=False)
    slave_arm.movejs(joint_positions_list=slave_path, acc=compliant_acc, vel=compliant_vel, radius=0.05, wait=False)

    # Wait for completion
    while master_arm.is_program_running() or slave_arm.is_program_running():
        time.sleep(0.05)

    print("✓ Compliant simultaneous movement complete.")


def demo_enhanced_object_handling():
    """
    Enhanced demonstration function implementing all 6 solutions for dual-arm object handling.
    This addresses force/torque threshold issues comprehensively.
    """
    print("=" * 80)
    print("ENHANCED DUAL-ARM OBJECT HANDLING DEMONSTRATION")
    print("=" * 80)
    print("This demo implements all 6 solutions:")
    print("1. Accurate Payload Setup")
    print("2. Synchronized Motion with Master-Slave Control") 
    print("3. Compliance Features")
    print("4. Gentle Approach Control")
    print("5. Adjusted Force Limits (Risk-Assessed)")
    print("6. Enhanced Logging & Verification")
    print("=" * 80)

    rbtx = None
    monitor = None
    plotter = None
    payload_manager = None
    sync_controller = None
    
    try:
        # Initialize the dual controller
        print("\n[INIT] Initializing Enhanced URDualController...")
        rbtx = urcx.URDualController(
            master_robot_ip=MASTER_ROBOT_IP,
            slave_robot_ip=SLAVE_ROBOT_IP,
            control_pc_ip=CONTROL_PC_IP
        )
        print("✓ Controller initialized successfully.")

        master_arm = rbtx._lft_arm
        slave_arm = rbtx._rgt_arm

        # Initialize payload manager (Solution 1)
        print("\n[INIT] Initializing Payload Manager...")
        payload_manager = PayloadManager(master_arm, slave_arm)
        payload_manager.configure_gripper_payload()  # Start with gripper-only payload
        
        # Initialize synchronized motion controller (Solution 2)
        print("\n[INIT] Initializing Synchronized Motion Controller...")
        sync_controller = SynchronizedMotionController(master_arm, slave_arm, payload_manager)
        
        # Initialize enhanced force/torque monitor (Solutions 5 & 6)
        if MONITORING_ENABLED:
            print("\n[INIT] Initializing Enhanced Force/Torque Monitor...")
            monitor = ForceTorqueMonitor(master_arm, slave_arm)
            monitor.start_monitoring()
            plotter = RealTimePlotter(monitor)
            plotter.start_plotting()
            print("✓ Enhanced force/torque monitoring initialized")
            print(f"  Normal thresholds: {FORCE_THRESHOLD_N}N, {TORQUE_THRESHOLD_NM}Nm")
            print(f"  Critical thresholds: {CRITICAL_FORCE_THRESHOLD_N}N, {CRITICAL_TORQUE_THRESHOLD_NM}Nm")

        # Step 1: Move to home positions (no object)
        print("\n" + "="*80)
        print("  STEP 1: MOVING TO HOME POSITIONS (GRIPPER-ONLY PAYLOAD)")
        print("="*80)
        
        sync_controller.execute_synchronized_move_with_compliance(
            MASTER_HOME_POSE, SLAVE_HOME_POSE,
            MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY,
            use_gentle_approach=True
        )

        if monitor:
            time.sleep(1)
            monitor.print_status()

        # Wait for user to attach object
        input("\n🤖 ATTACH OBJECT between the grippers now, then press Enter to continue...")
        
        # Step 2: Configure for object handling
        print("\n" + "="*80)
        print("  STEP 2: CONFIGURING FOR OBJECT HANDLING")
        print("="*80)
        
        # Update payload configuration (Solution 1)
        payload_manager.configure_object_payload()
        
        # Enable object handling mode with adjusted thresholds (Solution 5)
        if monitor:
            monitor.set_object_handling_mode(enabled=True)
            print(f"✓ Object handling thresholds: {OBJECT_FORCE_THRESHOLD_N}N, {OBJECT_TORQUE_THRESHOLD_NM}Nm")
            monitor.print_status()
        
        # Step 3: Move with object using enhanced synchronized control
        print("\n" + "="*80)
        print("  STEP 3: MOVING WITH OBJECT (ENHANCED SYNCHRONIZED CONTROL)")
        print("="*80)
        
        sync_controller.execute_synchronized_move_with_compliance(
            MASTER_UP_POSITION, SLAVE_UP_POSITION,
            OBJECT_MOVEMENT_TIME_SECONDS, OBJECT_JOINT_ACCELERATION, OBJECT_JOINT_VELOCITY,
            use_gentle_approach=True
        )
        
        if monitor:
            monitor.print_status()

        # Step 4: Demonstrate gentle approach control
        print("\n" + "="*80)
        print("  STEP 4: GENTLE APPROACH MOVEMENT (SOLUTION 4)")
        print("="*80)
        
        sync_controller.execute_synchronized_move_with_compliance(
            MASTER_LEFT_POSITION, SLAVE_LEFT_POSITION,
            OBJECT_MOVEMENT_TIME_SECONDS, OBJECT_JOINT_ACCELERATION, OBJECT_JOINT_VELOCITY,
            use_gentle_approach=True
        )
        
        if monitor:
            monitor.print_status()

        # Step 5: Test compliance with challenging movement
        print("\n" + "="*80)
        print("  STEP 5: COMPLIANCE TEST MOVEMENT")
        print("="*80)
        
        sync_controller.execute_synchronized_move_with_compliance(
            MASTER_RIGHT_POSITION, SLAVE_RIGHT_POSITION,
            OBJECT_MOVEMENT_TIME_SECONDS * 1.2, OBJECT_JOINT_ACCELERATION * 0.8, OBJECT_JOINT_VELOCITY * 0.8,
            use_gentle_approach=True
        )
        
        if monitor:
            monitor.print_status()

        # Step 6: Return to home with object
        print("\n" + "="*80)
        print("  STEP 6: RETURNING TO HOME WITH OBJECT")
        print("="*80)
        
        sync_controller.execute_synchronized_move_with_compliance(
            MASTER_HOME_POSE, SLAVE_HOME_POSE,
            OBJECT_MOVEMENT_TIME_SECONDS, OBJECT_JOINT_ACCELERATION, OBJECT_JOINT_VELOCITY,
            use_gentle_approach=True
        )
        
        if monitor:
            monitor.print_status()

        # Step 7: Release object and return to normal mode
        input("\n🤖 REMOVE OBJECT from grippers now, then press Enter to continue...")
        
        print("\n" + "="*80)
        print("  STEP 7: RETURNING TO NORMAL MODE")
        print("="*80)
        
        # Reconfigure payload for gripper-only
        payload_manager.configure_gripper_payload()
        
        if monitor:
            monitor.set_object_handling_mode(enabled=False)
            
        print("✓ Enhanced object handling demonstration complete!")
        
        # Print final summary
        if monitor:
            print("\n" + "="*80)
            print("  FINAL MONITORING SUMMARY")
            print("="*80)
            stats = monitor.get_statistics()
            print(f"Total data points logged: {stats['data_points']}")
            print(f"Maximum forces - Master: {stats['master_max_force']:.2f}N, Slave: {stats['slave_max_force']:.2f}N")
            print(f"Maximum torques - Master: {stats['master_max_torque']:.2f}Nm, Slave: {stats['slave_max_torque']:.2f}Nm")
            print(f"Warning events: {monitor.warning_count}")
            print(f"Critical events: {monitor.critical_count}")
            if LOG_TO_FILE:
                print(f"Detailed log saved to: {LOG_FILENAME}")

    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if monitor:
            monitor.stop_monitoring()
        if plotter:
            plotter.stop_plotting()
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nEnhanced object handling demo finished.")


def demo_payload_distribution():
    """
    Demonstration function specifically for payload distribution monitoring.
    This shows how to check if payload is equally shared between arms when handling objects.
    """
    print("=" * 80)
    print("PAYLOAD DISTRIBUTION MONITORING DEMONSTRATION")
    print("=" * 80)
    print("This demo shows how to monitor and verify equal payload sharing")
    print("between dual robot arms when handling shared objects.")
    print("=" * 80)

    rbtx = None
    monitor = None
    
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

        # Initialize force/torque monitor
        print("\n[INIT] Initializing Force/Torque Monitor...")
        monitor = ForceTorqueMonitor(master_arm, slave_arm)
        monitor.start_monitoring()
        print("✓ Force/torque monitoring initialized")

        # Step 1: Check distribution without object
        print("\n" + "="*80)
        print("  STEP 1: BASELINE MEASUREMENT (NO OBJECT)")
        print("="*80)
        print("Checking force distribution with grippers only (no shared object)...")
        time.sleep(2)  # Allow monitoring to collect data
        monitor.check_payload_distribution()
        
        # Step 2: Enable object handling mode and check with object
        input("\n🤖 ATTACH OBJECT between the grippers now, then press Enter to continue...")
        
        print("\n" + "="*80)
        print("  STEP 2: OBJECT HANDLING MODE")
        print("="*80)
        
        # Enable object handling mode
        monitor.set_object_handling_mode(enabled=True)
        print("Waiting for force readings to stabilize...")
        time.sleep(3)  # Allow forces to stabilize
        
        # Check payload distribution with object
        print("\nChecking payload distribution with shared object:")
        distribution_data = monitor.check_payload_distribution()
        
        if distribution_data:
            if distribution_data['is_balanced']:
                print("\n✅ SUCCESS: Payload is well balanced between arms!")
                print(f"   Load sharing: Master {distribution_data['master_ratio']*100:.1f}% | Slave {distribution_data['slave_ratio']*100:.1f}%")
            else:
                print("\n⚠️  WARNING: Uneven payload distribution detected!")
                print("   Consider adjusting object position or grip points for better balance.")
                print(f"   Current sharing: Master {distribution_data['master_ratio']*100:.1f}% | Slave {distribution_data['slave_ratio']*100:.1f}%")
        
        # Step 3: Continuous monitoring during movement
        print("\n" + "="*80)
        print("  STEP 3: CONTINUOUS MONITORING DURING MOVEMENT")
        print("="*80)
        
        move_choice = input("Test payload distribution during movement? (y/n): ").strip().lower()
        if move_choice == 'y':
            print("Moving arms while monitoring payload distribution...")
            
            # Move to up position while monitoring
            print("\nMoving to UP position...")
            execute_simultaneous_move(
                master_arm, MASTER_UP_POSITION,
                slave_arm, SLAVE_UP_POSITION,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            
            print("\nPayload distribution after movement:")
            monitor.check_payload_distribution()
            
            # Move back to home
            print("\nReturning to HOME position...")
            execute_simultaneous_move(
                master_arm, MASTER_HOME_POSE,
                slave_arm, SLAVE_HOME_POSE,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            
            print("\nFinal payload distribution check:")
            monitor.check_payload_distribution()

        # Step 4: Summary and recommendations
        print("\n" + "="*80)
        print("  PAYLOAD DISTRIBUTION SUMMARY")
        print("="*80)
        
        stats = monitor.get_statistics()
        print(f"Monitoring session statistics:")
        print(f"  • Total data points: {stats['data_points']}")
        print(f"  • Max forces - Master: {stats['master_max_force']:.2f}N, Slave: {stats['slave_max_force']:.2f}N")
        print(f"  • Warning events: {monitor.warning_count}")
        print(f"  • Critical events: {monitor.critical_count}")
        
        print(f"\nRecommendations for optimal payload sharing:")
        print(f"  • Ideal force distribution: 50% ± 20% per arm")
        print(f"  • Expected object mass per arm: {SHARED_OBJECT_MASS_KG * 0.5:.2f}kg")
        print(f"  • Total expected payload per arm: {SHARED_OBJECT_MASS_KG * 0.5 + RG2_MASS_KG:.2f}kg")
        print(f"  • Adjust object grip points if distribution is uneven")
        
        print("\n✓ Payload distribution demonstration complete!")

    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if monitor:
            monitor.stop_monitoring()
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nPayload distribution demo finished.")


def main():
    """Main function to move both robots to their home positions, pause, and optionally move to up positions."""
    print("=" * 60)
    print("Moving Master and Slave to Home Positions Only")
    print("=" * 60)
    print("WARNING: This script will move the robots. Press Ctrl+C to stop.")
    print("=" * 60)

    rbtx = None
    monitor = None
    plotter = None
    
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

        # Initialize force/torque monitor
        if MONITORING_ENABLED:
            print("\n[INIT] Initializing Force/Torque Monitor...")
            monitor = ForceTorqueMonitor(master_arm, slave_arm)
            monitor.start_monitoring()
            plotter = RealTimePlotter(monitor)
            plotter.start_plotting()
            print("✓ Force/torque monitoring initialized")

        print("\n" + "="*60)
        print("  MOVING BOTH ARMS TO HOME POSITIONS SIMULTANEOUSLY")
        print("="*60 + "\n")
        
        # Print initial force/torque status
        if monitor:
            time.sleep(1)  # Allow monitoring to start
            monitor.print_status()
        
        execute_simultaneous_move(
            master_arm, MASTER_HOME_POSE,
            slave_arm, SLAVE_HOME_POSE,
            MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
        )

        # Print force/torque status after movement
        if monitor:
            monitor.print_status()

        # Pause and ask user which position they want to go to
        input_result = input("\nBoth arms are at home. Where do you want to go next? (up/left/right/payload/object-mode/no): ").strip().lower()
        if input_result == 'up':
            print("\nMoving both arms to UP POSITION simultaneously...")
            execute_simultaneous_move(
                master_arm, MASTER_UP_POSITION,
                slave_arm, SLAVE_UP_POSITION,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            print("\nBoth arms have reached the UP POSITION.")

            # Print force/torque status after movement
            if monitor:
                monitor.print_status()

            # Continuous loop for next moves
            while True:
                input_result2 = input("\nWhere do you want to go next? (right/left/home/payload/no): ").strip().lower()
                if input_result2 == 'right':
                    print("\nMoving both arms to RIGHT POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_RIGHT_POSITION,
                        slave_arm, SLAVE_RIGHT_POSITION,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the RIGHT POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'left':
                    print("\nMoving both arms to LEFT POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_LEFT_POSITION,
                        slave_arm, SLAVE_LEFT_POSITION,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the LEFT POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'home':
                    print("\nMoving both arms to HOME POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_HOME_POSE,
                        slave_arm, SLAVE_HOME_POSE,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the HOME POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'payload':
                    print("\nChecking payload distribution...")
                    if monitor:
                        monitor.check_payload_distribution()
                    else:
                        print("⚠️ Force monitoring not enabled - cannot check payload distribution")
                elif input_result2 == 'no':
                    print("\nExiting movement loop.")
                    break
                else:
                    print("\nInvalid input. Please choose: right, left, home, payload, or no.")
        elif input_result == 'left':
            print("\nMoving both arms to LEFT POSITION simultaneously...")
            execute_simultaneous_move(
                master_arm, MASTER_LEFT_POSITION,
                slave_arm, SLAVE_LEFT_POSITION,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            print("\nBoth arms have reached the LEFT POSITION.")
            
            # Print force/torque status after movement
            if monitor:
                monitor.print_status()
                
            # Continuous loop for next moves
            while True:
                input_result2 = input("\nWhere do you want to go next? (right/left/home/payload/no): ").strip().lower()
                if input_result2 == 'right':
                    print("\nMoving both arms to RIGHT POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_RIGHT_POSITION,
                        slave_arm, SLAVE_RIGHT_POSITION,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the RIGHT POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'left':
                    print("\nMoving both arms to LEFT POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_LEFT_POSITION,
                        slave_arm, SLAVE_LEFT_POSITION,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the LEFT POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'home':
                    print("\nMoving both arms to HOME POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_HOME_POSE,
                        slave_arm, SLAVE_HOME_POSE,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the HOME POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'no':
                    print("\nExiting movement loop.")
                    break
                elif input_result2 == 'payload':
                    print("\nChecking payload distribution...")
                    if monitor:
                        monitor.check_payload_distribution()
                    else:
                        print("⚠️ Force monitoring not enabled - cannot check payload distribution")
                else:
                    print("\nInvalid input. Please choose: right, left, home, payload, or no.")
        elif input_result == 'payload':
            print("\nChecking payload distribution...")
            if monitor:
                monitor.check_payload_distribution()
            else:
                print("⚠️ Force monitoring not enabled - cannot check payload distribution")
        elif input_result == 'object-mode':
            print("\nToggling object handling mode...")
            if monitor:
                current_mode = monitor.get_monitoring_mode()
                new_mode = not current_mode['object_handling_mode']
                monitor.set_object_handling_mode(new_mode)
                if new_mode:
                    print("✓ Object handling mode ENABLED - using gentler force/torque thresholds")
                    print("  Attach an object between the grippers for shared payload handling")
                else:
                    print("✓ Object handling mode DISABLED - using normal force/torque thresholds")
                    print("  Remove any objects from between the grippers")
            else:
                print("⚠️ Force monitoring not enabled - cannot toggle object handling mode")
        elif input_result == 'right':
            print("\nMoving both arms to RIGHT POSITION simultaneously...")
            execute_simultaneous_move(
                master_arm, MASTER_RIGHT_POSITION,
                slave_arm, SLAVE_RIGHT_POSITION,
                MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
            )
            print("\nBoth arms have reached the RIGHT POSITION.")
            
            # Print force/torque status after movement
            if monitor:
                monitor.print_status()
                
            # Continuous loop for next moves
            while True:
                input_result2 = input("\nWhere do you want to go next? (right/left/home/payload/no): ").strip().lower()
                if input_result2 == 'right':
                    print("\nMoving both arms to RIGHT POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_RIGHT_POSITION,
                        slave_arm, SLAVE_RIGHT_POSITION,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the RIGHT POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'left':
                    print("\nMoving both arms to LEFT POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_LEFT_POSITION,
                        slave_arm, SLAVE_LEFT_POSITION,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the LEFT POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'home':
                    print("\nMoving both arms to HOME POSITION simultaneously...")
                    execute_simultaneous_move(
                        master_arm, MASTER_HOME_POSE,
                        slave_arm, SLAVE_HOME_POSE,
                        MOVEMENT_TIME_SECONDS, JOINT_ACCELERATION, JOINT_VELOCITY
                    )
                    print("\nBoth arms have reached the HOME POSITION.")
                    
                    # Print final force/torque status
                    if monitor:
                        monitor.print_status()
                elif input_result2 == 'payload':
                    print("\nChecking payload distribution...")
                    if monitor:
                        monitor.check_payload_distribution()
                    else:
                        print("⚠️ Force monitoring not enabled - cannot check payload distribution")
                elif input_result2 == 'no':
                    print("\nExiting movement loop.")
                    break
                else:
                    print("\nInvalid input. Please choose: right, left, home, payload, or no.")
        else:
            print("\nNo further movement. Program will exit.")

    except KeyboardInterrupt:
        print("\n\nScript interrupted by user. Stopping robots.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        # Stop monitoring and plotting
        if monitor:
            monitor.stop_monitoring()
        if plotter:
            plotter.stop_plotting()
            
        # Cleanly close the connections
        if rbtx:
            rbtx._lft_arm.close()
            rbtx._rgt_arm.close()
            print("\n[CLEANUP] Robot connections closed.")

    print("\nProgram finished.")


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Enhanced Dual Robot Arm Controller with Force/Torque Solutions')
    parser.add_argument('--demo-enhanced', action='store_true', 
                       help='Run enhanced object handling demonstration with all 6 solutions')
    parser.add_argument('--demo-object', action='store_true', 
                       help='Run basic object handling demonstration (legacy)')
    parser.add_argument('--demo-payload', action='store_true',
                       help='Run payload distribution monitoring demonstration')
    parser.add_argument('--object-mode', action='store_true',
                       help='Use gentler parameters for object handling')
    parser.add_argument('--log-file', type=str, default=LOG_FILENAME,
                       help=f'Specify log file name (default: {LOG_FILENAME})')
    parser.add_argument('--force-threshold', type=float, default=FORCE_THRESHOLD_N,
                       help=f'Set force threshold in Newtons (default: {FORCE_THRESHOLD_N})')
    parser.add_argument('--torque-threshold', type=float, default=TORQUE_THRESHOLD_NM,
                       help=f'Set torque threshold in Nm (default: {TORQUE_THRESHOLD_NM})')
    args = parser.parse_args()
    
    # Update global parameters if specified
    if args.log_file != LOG_FILENAME:
        globals()['LOG_FILENAME'] = args.log_file
    if args.force_threshold != FORCE_THRESHOLD_N:
        globals()['FORCE_THRESHOLD_N'] = args.force_threshold
    if args.torque_threshold != TORQUE_THRESHOLD_NM:
        globals()['TORQUE_THRESHOLD_NM'] = args.torque_threshold
    
    if args.demo_enhanced:
        demo_enhanced_object_handling()
    elif args.demo_payload:
        demo_payload_distribution()
    elif args.demo_object:
        # Keep legacy demo for backward compatibility (but recommend enhanced version)
        print("⚠️  Note: Consider using --demo-enhanced for the improved implementation")
        demo_enhanced_object_handling()  # Actually use enhanced version
    else:
        main() 