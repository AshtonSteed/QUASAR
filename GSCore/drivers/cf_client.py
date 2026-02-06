# Might not need, cflib wrappers for common stuff
# Sending 6DOF to UAV, changing setpoint, UAV parameters, logging functions
import queue
import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from GSCore.drivers.motive_client import start_motive_stream
from GSCore.drivers.mock_motive_client import start_mock_stream
from common_classes import SystemState, Pose

class CrazyflieDriver:
    def __init__(self, uri, pose_queue, command_queue):
        """
        uri: Radio URI
        pose_queue: The shared queue with NatNet data
        command_queue: The shared queue for sending setpoints
        """
        self.uri = uri
        self.pose_queue = pose_queue
        self.command_queue = command_queue
        self.cf = Crazyflie(rw_cache='./cache')
        
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
        self.start_position_logging() # Start logging position data from CF
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
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
        self.cf.extpos.send_extpose(mx, my, mz, qx, qy, qz, qw)
        
    from cflib.crazyflie.log import LogConfig

    def start_position_logging(self):
        # 1. Configure the Log Block
        # Period is how often the crazyflie sends data
        log_conf = LogConfig(name='Position', period_in_ms=50)

      
        # Add all variables to log from CF parameter list
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        log_conf.add_variable('pm.batteryLevel')
        

        # 3. Add the configuration to the Crazyflie
        try:
            self.cf.log.add_config(log_conf)
            
            # 4. Attach a callback function (What happens when data arrives?)
            log_conf.data_received_cb.add_callback(self.position_data_callback)
            
            # 5. Start the logging
            log_conf.start()
            print("Logging started!")
            
        except KeyError as e:
            print(f"Could not find variable in TOC: {e}")
        except AttributeError:
            print("Could not add log config, bad configuration.")
    
    # Function ran whenever log data is received from CF
    def position_data_callback(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        bat = data['pm.batteryLevel']
    
        # Example: Print only occasionally to avoid spamming console
        print(f"[{timestamp}] Pos: ({x:.2f}, {y:.2f}, {z:.2f}) | Battery: {bat}%")
    # ==========================================
    # THE CONTROL WORKER
    # ==========================================

    def _control_loop(self):
        """
        The main flight loop. Runs in background thread.
        """
        self._arm_kalman()
        
        while self.running:
            self._send_state()
            time.sleep(0.1) # 10Hz update rate
        
        
        

def test_cf_connection(uri):
    pose_queue = queue.Queue(maxsize=1)
    shared_state = SystemState()
    motive_client = start_mock_stream(pose_queue, shared_state)
    cflib.crtp.init_drivers()
    driver = CrazyflieDriver(uri, pose_queue, None)
    if driver.connect():
        print("Connection successful!")
        driver.start()
        time.sleep(10)  # Let it run for a bit
        driver.stop()
    else:
        print("Connection failed.")