import numpy as np
import time
import threading
from collections import deque
import logging

# Configuration constants
RG2_MASS_KG = 0.78
RG2_COG_OFFSET = [0.0, 0.0, 0.055]
SHARED_OBJECT_MASS_KG = 0.5
SHARED_OBJECT_COG = [0.0, 0.0, 0.1]

# Object detection thresholds
GRIPPER_OPEN_THRESHOLD_MM = 90.0    # Width above which gripper is considered open
GRIPPER_CLOSED_THRESHOLD_MM = 20.0  # Width below which gripper is considered closed
OBJECT_DETECTION_WIDTH_MIN = 25.0   # Minimum width indicating object presence
OBJECT_DETECTION_WIDTH_MAX = 85.0   # Maximum width indicating object presence

# Force/torque thresholds for object detection
BASELINE_FORCE_THRESHOLD = 5.0      # Baseline force when no object
OBJECT_FORCE_THRESHOLD = 15.0       # Force indicating object presence
FORCE_STABILITY_TIME = 2.0          # Time to wait for force stabilization

class EnhancedPayloadManager:
    """
    Enhanced payload manager with automatic object detection using multiple methods:
    1. Gripper width monitoring
    2. Force/torque analysis
    3. Manual state tracking
    4. Dual-arm coordination verification
    """
    
    def __init__(self, master_arm, slave_arm, master_gripper=None, slave_gripper=None):
        self.master_arm = master_arm
        self.slave_arm = slave_arm
        self.master_gripper = master_gripper
        self.slave_gripper = slave_gripper
        
        # State tracking
        self.object_attached = False
        self.detection_method = "manual"  # manual, gripper_width, force_torque, combined
        self.confidence_level = 0.0
        
        # Monitoring data
        self.force_history = deque(maxlen=100)
        self.width_history = deque(maxlen=50)
        self.detection_history = deque(maxlen=20)
        
        # Threading for continuous monitoring
        self.monitoring_active = False
        self.monitoring_thread = None
        
        # Logging
        self.logger = logging.getLogger(__name__)
        
    def start_monitoring(self):
        """Start continuous object detection monitoring."""
        if self.monitoring_active:
            return
            
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        self.logger.info("Enhanced payload monitoring started")
    
    def stop_monitoring(self):
        """Stop continuous monitoring."""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)
        self.logger.info("Enhanced payload monitoring stopped")
    
    def _monitoring_loop(self):
        """Continuous monitoring loop for automatic object detection."""
        while self.monitoring_active:
            try:
                # Method 1: Gripper width detection
                width_detection = self._detect_object_by_gripper_width()
                
                # Method 2: Force/torque analysis
                force_detection = self._detect_object_by_force_torque()
                
                # Method 3: Combined analysis
                combined_detection = self._combined_object_detection(width_detection, force_detection)
                
                # Update state if confidence is high enough
                if combined_detection['confidence'] > 0.7:
                    old_state = self.object_attached
                    self.object_attached = combined_detection['object_detected']
                    self.confidence_level = combined_detection['confidence']
                    self.detection_method = combined_detection['method']
                    
                    # If state changed, update payload configuration
                    if old_state != self.object_attached:
                        self._auto_configure_payload()
                        self.logger.info(f"Object state changed: {old_state} -> {self.object_attached} "
                                       f"(confidence: {self.confidence_level:.2f}, method: {self.detection_method})")
                
                time.sleep(0.5)  # Monitor at 2Hz
                
            except Exception as e:
                self.logger.error(f"Error in monitoring loop: {e}")
                time.sleep(1.0)
    
    def _detect_object_by_gripper_width(self):
        """Detect object presence using gripper width measurements."""
        if not (self.master_gripper and self.slave_gripper):
            return {'detected': False, 'confidence': 0.0, 'data': None}
        
        try:
            master_width = self.master_gripper.get_rg_width()
            slave_width = self.slave_gripper.get_rg_width()
            
            self.width_history.append({
                'timestamp': time.time(),
                'master_width': master_width,
                'slave_width': slave_width
            })
            
            # Analysis logic
            avg_width = (master_width + slave_width) / 2.0
            width_diff = abs(master_width - slave_width)
            
            # Object detection criteria
            object_detected = (
                OBJECT_DETECTION_WIDTH_MIN <= avg_width <= OBJECT_DETECTION_WIDTH_MAX and
                width_diff < 30.0  # Grippers should have similar width when holding object
            )
            
            # Confidence calculation
            if object_detected:
                # Higher confidence when widths are in optimal range and similar
                width_confidence = 1.0 - (width_diff / 30.0)  # Penalty for width difference
                range_confidence = 1.0 - abs(avg_width - 55.0) / 30.0  # Optimal around 55mm
                confidence = min(width_confidence * range_confidence, 1.0)
            else:
                confidence = 0.1 if avg_width < GRIPPER_CLOSED_THRESHOLD_MM else 0.0
            
            return {
                'detected': object_detected,
                'confidence': confidence,
                'data': {
                    'master_width': master_width,
                    'slave_width': slave_width,
                    'avg_width': avg_width,
                    'width_diff': width_diff
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error detecting object by gripper width: {e}")
            return {'detected': False, 'confidence': 0.0, 'data': None}
    
    def _detect_object_by_force_torque(self):
        """Detect object presence using force/torque measurements."""
        try:
            master_tcp_force = self.master_arm.get_tcp_force(wait=False)
            slave_tcp_force = self.slave_arm.get_tcp_force(wait=False)
            
            if master_tcp_force is None or slave_tcp_force is None:
                return {'detected': False, 'confidence': 0.0, 'data': None}
            
            # Calculate force magnitudes
            master_force = np.linalg.norm(master_tcp_force[:3])
            slave_force = np.linalg.norm(slave_tcp_force[:3])
            combined_force = (master_force + slave_force) / 2.0
            
            self.force_history.append({
                'timestamp': time.time(),
                'master_force': master_force,
                'slave_force': slave_force,
                'combined_force': combined_force
            })
            
            # Object detection based on force patterns
            object_detected = combined_force > OBJECT_FORCE_THRESHOLD
            
            # Confidence based on force stability and magnitude
            if len(self.force_history) >= 10:
                recent_forces = [f['combined_force'] for f in list(self.force_history)[-10:]]
                force_stability = 1.0 - (np.std(recent_forces) / max(np.mean(recent_forces), 1.0))
                force_magnitude_confidence = min(combined_force / OBJECT_FORCE_THRESHOLD, 2.0) / 2.0
                confidence = force_stability * force_magnitude_confidence if object_detected else 0.1
            else:
                confidence = 0.3 if object_detected else 0.1
            
            return {
                'detected': object_detected,
                'confidence': min(confidence, 1.0),
                'data': {
                    'master_force': master_force,
                    'slave_force': slave_force,
                    'combined_force': combined_force
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error detecting object by force/torque: {e}")
            return {'detected': False, 'confidence': 0.0, 'data': None}
    
    def _combined_object_detection(self, width_detection, force_detection):
        """Combine multiple detection methods for robust object detection."""
        detections = [width_detection, force_detection]
        valid_detections = [d for d in detections if d['confidence'] > 0.0]
        
        if not valid_detections:
            return {'object_detected': False, 'confidence': 0.0, 'method': 'none'}
        
        # Weighted combination of detection methods
        width_weight = 0.7  # Gripper width is more reliable
        force_weight = 0.3
        
        combined_confidence = (
            width_detection['confidence'] * width_weight +
            force_detection['confidence'] * force_weight
        )
        
        # Object is detected if either method has high confidence or both agree
        object_detected = (
            (width_detection['detected'] and width_detection['confidence'] > 0.6) or
            (force_detection['detected'] and force_detection['confidence'] > 0.6) or
            (width_detection['detected'] and force_detection['detected'] and combined_confidence > 0.4)
        )
        
        # Determine primary detection method
        if width_detection['confidence'] > force_detection['confidence']:
            primary_method = 'gripper_width'
        elif force_detection['confidence'] > width_detection['confidence']:
            primary_method = 'force_torque'
        else:
            primary_method = 'combined'
        
        return {
            'object_detected': object_detected,
            'confidence': combined_confidence,
            'method': primary_method,
            'width_data': width_detection['data'],
            'force_data': force_detection['data']
        }
    
    def _auto_configure_payload(self):
        """Automatically configure payload based on detected object state."""
        if self.object_attached:
            self.configure_object_payload()
        else:
            self.configure_gripper_payload()
    
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
        master_payload_mass = RG2_MASS_KG + (SHARED_OBJECT_MASS_KG * 0.5)
        slave_payload_mass = RG2_MASS_KG + (SHARED_OBJECT_MASS_KG * 0.5)
        
        self.master_arm.set_payload(master_payload_mass, RG2_COG_OFFSET)
        self.slave_arm.set_payload(slave_payload_mass, RG2_COG_OFFSET)
        self.object_attached = True
        print(f"  Master payload: {master_payload_mass:.2f}kg at CoG {RG2_COG_OFFSET}")
        print(f"  Slave payload:  {slave_payload_mass:.2f}kg at CoG {RG2_COG_OFFSET}")
        print("  ⚠️  Each arm configured for 50% of shared object mass")
    
    def manual_set_object_state(self, object_attached, configure_payload=True):
        """Manually set object attachment state."""
        old_state = self.object_attached
        self.object_attached = object_attached
        self.detection_method = "manual"
        self.confidence_level = 1.0
        
        if configure_payload:
            self._auto_configure_payload()
        
        self.logger.info(f"Manual object state change: {old_state} -> {self.object_attached}")
    
    def get_detection_status(self):
        """Get detailed object detection status."""
        return {
            'object_attached': self.object_attached,
            'detection_method': self.detection_method,
            'confidence_level': self.confidence_level,
            'monitoring_active': self.monitoring_active,
            'last_width_data': self.width_history[-1] if self.width_history else None,
            'last_force_data': self.force_history[-1] if self.force_history else None
        }
    
    def get_current_payload_info(self):
        """Get current payload configuration with detection info."""
        master_mass = (RG2_MASS_KG + (SHARED_OBJECT_MASS_KG * 0.5)) if self.object_attached else RG2_MASS_KG
        slave_mass = (RG2_MASS_KG + (SHARED_OBJECT_MASS_KG * 0.5)) if self.object_attached else RG2_MASS_KG
        
        return {
            'object_attached': self.object_attached,
            'master_mass': master_mass,
            'slave_mass': slave_mass,
            'shared_object_mass': SHARED_OBJECT_MASS_KG if self.object_attached else 0.0,
            'detection_method': self.detection_method,
            'confidence_level': self.confidence_level,
            'gripper_data': self.width_history[-1] if self.width_history else None,
            'force_data': self.force_history[-1] if self.force_history else None
        }
    
    def print_status(self):
        """Print comprehensive status information."""
        status = self.get_detection_status()
        payload_info = self.get_current_payload_info()
        
        print("\n--- Enhanced Payload Manager Status ---")
        print(f"Object Attached: {status['object_attached']}")
        print(f"Detection Method: {status['detection_method']}")
        print(f"Confidence Level: {status['confidence_level']:.2f}")
        print(f"Monitoring Active: {status['monitoring_active']}")
        
        if status['last_width_data']:
            width_data = status['last_width_data']
            print(f"Gripper Widths: Master={width_data['master_width']:.1f}mm, "
                  f"Slave={width_data['slave_width']:.1f}mm")
        
        if status['last_force_data']:
            force_data = status['last_force_data']
            print(f"Forces: Master={force_data['master_force']:.1f}N, "
                  f"Slave={force_data['slave_force']:.1f}N")
        
        print(f"Payload Masses: Master={payload_info['master_mass']:.2f}kg, "
              f"Slave={payload_info['slave_mass']:.2f}kg") 