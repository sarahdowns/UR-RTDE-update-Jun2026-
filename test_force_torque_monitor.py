#!/usr/bin/env python3
"""
Test script for force/torque monitoring with MATLAB visualization.
This script simulates force/torque data to test the MATLAB communication.
"""

import numpy as np
import time
import socket
import json
import threading
import math

# Configuration
MATLAB_HOST = "localhost"
MATLAB_PORT = 12345
DATA_SEND_FREQUENCY_HZ = 10

class SimulatedForceTorqueMonitor:
    def __init__(self, matlab_host="localhost", matlab_port=12345):
        self.matlab_host = matlab_host
        self.matlab_port = matlab_port
        self.matlab_socket = None
        self.is_monitoring = False
        
    def connect_to_matlab(self):
        """Establish connection to MATLAB for data transmission."""
        try:
            self.matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.matlab_socket.connect((self.matlab_host, self.matlab_port))
            print(f"✓ Connected to MATLAB at {self.matlab_host}:{self.matlab_port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to MATLAB: {e}")
            print("Make sure MATLAB is running and listening on the specified port.")
            return False
    
    def send_data_to_matlab(self, data):
        """Send data to MATLAB via TCP socket."""
        if self.matlab_socket:
            try:
                json_data = json.dumps(data)
                self.matlab_socket.send((json_data + '\n').encode())
            except Exception as e:
                print(f"✗ Error sending data to MATLAB: {e}")
                self.matlab_socket = None
    
    def simulate_force_torque_data(self, duration=10.0):
        """Simulate force and torque data for testing."""
        print(f"\n--- Simulating Force/Torque Data for {duration} seconds ---")
        self.is_monitoring = True
        start_time = time.time()
        monitoring_interval = 1.0 / DATA_SEND_FREQUENCY_HZ
        
        try:
            while self.is_monitoring:
                current_time = time.time() - start_time
                
                if current_time >= duration:
                    break
                
                # Simulate realistic force/torque data with some variation
                time_factor = current_time / duration
                
                # Master arm simulation (more variation)
                master_force_base = 15.0 + 5.0 * math.sin(2 * math.pi * current_time * 0.5)
                master_force_noise = np.random.normal(0, 2.0)
                master_force = max(0, master_force_base + master_force_noise)
                
                master_torque_base = 3.0 + 1.5 * math.sin(2 * math.pi * current_time * 0.3)
                master_torque_noise = np.random.normal(0, 0.5)
                master_torque = max(0, master_torque_base + master_torque_noise)
                
                # Slave arm simulation (different pattern)
                slave_force_base = 12.0 + 3.0 * math.cos(2 * math.pi * current_time * 0.4)
                slave_force_noise = np.random.normal(0, 1.5)
                slave_force = max(0, slave_force_base + slave_force_noise)
                
                slave_torque_base = 2.5 + 1.0 * math.cos(2 * math.pi * current_time * 0.6)
                slave_torque_noise = np.random.normal(0, 0.3)
                slave_torque = max(0, slave_torque_base + slave_torque_noise)
                
                # Simulate joint positions (6 DOF)
                master_joints = [
                    0.1 * math.sin(current_time * 0.5),
                    -1.5 + 0.2 * math.cos(current_time * 0.3),
                    -1.8 + 0.1 * math.sin(current_time * 0.7),
                    -1.0 + 0.15 * math.cos(current_time * 0.4),
                    1.5 + 0.1 * math.sin(current_time * 0.6),
                    0.0 + 0.05 * math.cos(current_time * 0.8)
                ]
                
                slave_joints = [
                    0.05 * math.sin(current_time * 0.6),
                    -2.1 + 0.15 * math.cos(current_time * 0.4),
                    2.0 + 0.1 * math.sin(current_time * 0.5),
                    -2.2 + 0.12 * math.cos(current_time * 0.3),
                    -1.5 + 0.08 * math.sin(current_time * 0.7),
                    -0.03 + 0.03 * math.cos(current_time * 0.9)
                ]
                
                # Prepare data for MATLAB
                matlab_data = {
                    'timestamp': current_time,
                    'master': {
                        'force': master_force,
                        'torque': master_torque,
                        'joints': master_joints
                    },
                    'slave': {
                        'force': slave_force,
                        'torque': slave_torque,
                        'joints': slave_joints
                    }
                }
                
                # Send data to MATLAB
                self.send_data_to_matlab(matlab_data)
                
                # Print status every 2 seconds
                if int(current_time * DATA_SEND_FREQUENCY_HZ) % (2 * DATA_SEND_FREQUENCY_HZ) == 0:
                    print(f"⏱️  {current_time:.1f}s - Master: {master_force:.2f}N, {master_torque:.2f}Nm | "
                          f"Slave: {slave_force:.2f}N, {slave_torque:.2f}Nm")
                
                time.sleep(monitoring_interval)
                
        except KeyboardInterrupt:
            print("\n⚠️  Simulation interrupted by user.")
        
        self.is_monitoring = False
        print("✓ Force/torque simulation completed.")
    
    def close(self):
        """Clean up connections."""
        self.is_monitoring = False
        if self.matlab_socket:
            self.matlab_socket.close()
        print("✓ Connections closed.")


def main():
    """Main function to test force/torque monitoring with simulated data."""
    print("=" * 60)
    print("Force/Torque Monitoring Test with Simulated Data")
    print("=" * 60)
    print("This script simulates force/torque data for testing MATLAB visualization.")
    print("=" * 60)
    
    monitor = None
    try:
        # Initialize the monitor
        monitor = SimulatedForceTorqueMonitor(
            matlab_host=MATLAB_HOST,
            matlab_port=MATLAB_PORT
        )
        
        # Connect to MATLAB
        if not monitor.connect_to_matlab():
            print("Continuing without MATLAB connection...")
        
        # Simulate data for 10 seconds
        monitor.simulate_force_torque_data(duration=10.0)
        
        print("\nTest completed successfully!")
        print("Check the MATLAB window for real-time plots.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n✗ An error occurred: {e}")
    finally:
        if monitor:
            monitor.close()
        print("\nTest finished.")


if __name__ == "__main__":
    main() 