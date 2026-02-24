# Might not need, cflib wrappers for common stuff
# Sending 6DOF to UAV, changing setpoint, UAV parameters, logging functions
import math
import queue
import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils.encoding import decompress_quaternion
from GSCore.drivers.motive_client import start_motive_stream
from GSCore.drivers.mock_motive_client import start_mock_stream
from common_classes import SystemState, Pose

class CrazyflieDriver:
    def __init__(self, uri, pose_queue, command_queue, shared_state=None):
        """
        uri: Radio URI
        pose_queue: The shared queue with NatNet data
        command_queue: The shared queue for sending setpoints
        """
        self.uri = uri
        self.pose_queue = pose_queue
        self.command_queue = command_queue
        self.logging_state = shared_state
        self.cf = Crazyflie(rw_cache='./cache')
        self.scf = SyncCrazyflie(uri, cf=self.cf)
        
        # --- Threading Events
        self.is_connected = threading.Event()
        self.is_disconnected = threading.Event()
        self.connection_error = False

        # --- Attach Callbacks for connection events ---
        self.cf.connected.add_callback(self._connected_cb)
        self.cf.disconnected.add_callback(self._disconnected_cb)
        self.cf.connection_failed.add_callback(self._connection_failed_cb)
        self.cf.connection_lost.add_callback(self._connection_lost_cb)

        # --- Control State ---
        self.running = False
        self.control_thread = None
        self.update_thread = None

    
    def connect(self):
        """
        Blocks until connection is established. 
        Returns True if successful, False if failed.
        """
        print(f"Connecting to {self.uri}...")
        self.cf.open_link(self.uri)
        
        # Wait here until the callback fires (or 10s timeout)
        # This makes the code 'feel' synchronous and simple.
        if not self.is_connected.wait(timeout=10.0):
            print("Connection timed out!")
            self.cf.close_link()
            return False
            
        if self.connection_error:
            return False
            
        return True

    def start(self):
        """Starts the control loop worker thread"""
        if not self.is_connected.is_set():
            print("Cannot start: Not connected.")
            return

    
        self.running = True
        self._arm_kalman()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        self.update_thread = threading.Thread(target=self._pose_sender_loop, daemon=True)
        self.update_thread.start() # Start the thread that continuously sends pose updates to the CF
        print("Control loop started.")

    def stop(self):
        """Stops the drone and disconnects"""
        self.running = False
        if self.control_thread:
            self.control_thread.join()
        
        # Safe disconnect
        self.cf.close_link()
        self.is_disconnected.wait(timeout=5.0)

    # Callbacks for CF events, dont call these directly
    def _connected_cb(self, link_uri):
        print(f"Connected to {link_uri}")
        self.is_connected.set() # <--- Unblocks the connect() function!

    def _disconnected_cb(self, link_uri):
        print(f"Disconnected from {link_uri}")
        self.is_connected.clear()
        self.is_disconnected.set()

    def _connection_failed_cb(self, link_uri, msg):
        print(f"Connection Failed: {msg}")
        self.connection_error = True
        self.is_connected.set() # Unblock so connect() can return False

    def _connection_lost_cb(self, link_uri, msg):
        print(f"Connection Lost: {msg}")
        
    # Kalman Filter Arming
    
    def _arm_kalman(self, err=1e-3):
        """
        Enables Kalman Filter on crazyflie, allowing for external pose updates from Motive.
        """
        # Enable EKF and set position error 
        print("Configuring EKF parameters...")
        # Use Kalman state estimator
        self.cf.param.set_value('stabilizer.estimator', '2') # EKF
        # Set the expected position error from Motive [mm]
        self.cf.param.set_value('locSrv.extPosStdDev', err)
        # Set the expected attitude error from Motive [degrees]
        self.cf.param.set_value('locSrv.extQuatStdDev', 5 * err)
        
        pose = self._extract_pose_from_queue()
        if pose is None:
            print("No valid pose data available to arm Kalman filter.")
            return
        
        x, y, z = pose.x, pose.y, pose.z
        
        
        self.cf.param.set_value('kalman.initialX', x)
        self.cf.param.set_value('kalman.initialY', y)
        self.cf.param.set_value('kalman.initialZ', z)
        
        self.cf.param.set_value('kalman.resetEstimation', '1')
        
        time.sleep(0.5) # Wait for CF to start up EKF
        
        self.cf.param.set_value('kalman.resetEstimation', '0')
        
        print("EKF armed with initial position from Motive.")
        
    def _extract_pose_from_queue(self):
        """
        Non-blocking check for new pose data from Motive. Returns None if no new data.
        """
        try:
            # Try to get most recent pose stored from motive
            packet = self.pose_queue.get_nowait()
            if packet.valid:
                # If tracking is valid, return the pose data
                return packet
            else:
                pass
        except queue.Empty:  
            pass
        # If invalid or motive isnt sending, return None
        return None

    def _send_state(self):
        # Send latest pose data if it exists, otherwise do nothing
        pose = self._extract_pose_from_queue()
        if pose is None:
            # Invalid position, do not update state estimation or commands
            return
        mx, my, mz, qx, qy, qz, qw = pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw
        #self.cf.extpos.send_extpose(mx, my, mz, qx, qy, qz, qw)
        self.cf.extpos.send_extpos(mx, my, mz)
        
    from cflib.crazyflie.log import LogConfig

    def start_logging(self):
        
        # Position Logging, High frequency logging of drone position and rotation from UAV estimator
        pose_log = LogConfig(name='Pose', period_in_ms=15) # 67Hz, faster logging

        # Add all variables to log from CF parameter list
        pose_log.add_variable('stateEstimateZ.x', 'int16_t')
        pose_log.add_variable('stateEstimateZ.y', 'int16_t')
        pose_log.add_variable('stateEstimateZ.z', 'int16_t')
        pose_log.add_variable('stateEstimateZ.quat', 'uint32_t')
    
        # Dynamics Logging, Moderate frequency logging of velocity, acceleration, and angular rates from UAV estimator
        dyn_log = LogConfig(name='Dynamics', period_in_ms=25) # 40Hz, moderate logging of estimated dynamics
        dyn_log.add_variable('stateEstimateZ.vx', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.vy', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.vz', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.ax', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.ay', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.az', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        dyn_log.add_variable('stateEstimateZ.rateYaw', 'int16_t')
        
        # Motor logging, Moderate frequency logging of motor setpoints for control debugging
        motor_log = LogConfig(name='Control', period_in_ms=25) # 40Hz, moderate logging of motor stepoints
        motor_log.add_variable('motor.m1', 'uint16_t')
        motor_log.add_variable('motor.m2', 'uint16_t')
        motor_log.add_variable('motor.m3', 'uint16_t')
        motor_log.add_variable('motor.m4', 'uint16_t')

        # Health Logging, Low frequency logging of battery and UAV status (flying, crash, etc)
        health_log = LogConfig(name='Health', period_in_ms=250) # 4Hz, low frequency logging of battery and UAV Status
        health_log.add_variable('supervisor.info', 'uint16_t') # See CF docs for state codes
        health_log.add_variable('pm.vbatMV', 'uint16_t') # Battery voltage in mV
        
        # 3. Add the configuration to the Crazyflie
        try:
            self.cf.log.add_config(pose_log)
            self.cf.log.add_config(dyn_log)
            self.cf.log.add_config(motor_log)
            self.cf.log.add_config(health_log)
            
           # add callback functions from the shared logging state
            pose_log.data_received_cb.add_callback(self.state.pose_data_callback)
            dyn_log.data_received_cb.add_callback(self.state.dyn_data_callback)
            motor_log.data_received_cb.add_callback(self.state.motor_data_callback)
            health_log.data_received_cb.add_callback(self.state.health_data_callback)
            
            # 5. Start the logging
            pose_log.start()
            dyn_log.start()
            motor_log.start()
            health_log.start()
            print("Logging started!")
            
        except KeyError as e:
            print(f"Could not find variable in TOC: {e}")
        except AttributeError:
            print("Could not add log config, bad configuration.")
    
   
    
    def _pose_sender_loop(self):
        """
        Runs continuously to feed Mocap data to the drone.
        Must run INDEPENDENTLY of the flight logic.
        """
        print("Pose sender started.")
        while self.running:
            pose = self._extract_pose_from_queue()
            if pose and pose.valid:
                # Send external position to CF
                self.cf.extpos.send_extpose(
                    pose.x, pose.y, pose.z,
                    pose.qx, pose.qy, pose.qz, pose.qw
                )
            # 30Hz - 50Hz update rate is standard for Mocap
            time.sleep(0.01)
    # ==========================================
    # THE CONTROL WORKER
    # ==========================================

    def _control_loop(self):
        """
        The main flight loop. Runs in background thread.
        """
        self.start_logging() # Start logging position data from CF
        self.cf.platform.send_arming_request(True)
        
        i = 0
        

        with MotionCommander(self.scf) as mc:
            mc.up(0.5)
            time.sleep(2)
            mc.stop()
        
        
       
        
        
        

def test_cf_connection(uri, shared_state=None):
    pose_queue = queue.Queue(maxsize=1)
    shared_state = SystemState() if shared_state is None else shared_state
    motive_client = start_motive_stream(pose_queue, shared_state)
    cflib.crtp.init_drivers()
    driver = CrazyflieDriver(uri, pose_queue, shared_state)
    if driver.connect():
        print("Connection successful!")
        driver.start()
        time.sleep(10)  # Let it run for a bit
        driver.stop()
    else:
        print("Connection failed.")
    return driver

def connect_to_uav(uri, pose_queue=None, command_queue=None, shared_state=None):
    pose_queue = queue.Queue(maxsize=1) if pose_queue is None else pose_queue
    command_queue = queue.Queue(maxsize=1) if command_queue is None else command_queue
    shared_state = SystemState() if shared_state is None else shared_state
    #motive_client = start_motive_stream(pose_queue, shared_state)
    cflib.crtp.init_drivers()
    driver = CrazyflieDriver(uri, pose_queue, command_queue, shared_state=shared_state)
    if driver.connect():
        print("Connection successful!")
        driver.start()
        time.sleep(10)  # Let it run for a bit
        driver.stop()
    else:
        print("Connection failed.")
    return driver
    