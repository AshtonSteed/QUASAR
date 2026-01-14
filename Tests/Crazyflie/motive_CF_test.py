import logging
import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from natnet import NatNetClient, DataFrame

# --- Configuration ---
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
URI = 'radio://0/80/2M/E7E7E7E7E7'
MOTIVE_IP = "127.0.0.1"
RIGID_BODY_ID = 1

# Global variable for the latest position (Thread-safe)
latest_pose = None

# --- Motive Callback (Updates the global variable) ---
def receive_motive_data(data_frame: DataFrame):
    global latest_pose
    for body in data_frame.rigid_bodies:
        if body.id_num == RIGID_BODY_ID:
            pos = body.pos
            rot = body.rot
            
            # 
            # --- CRITICAL COORDINATE TRANSFORMATION ---
            # Motive is usually Y-Up (Left Handed). Crazyflie is Z-Up (Right Handed).
            # If your drone flips instantly, Check these axes!
            
            # Standard Mapping:
            cf_x = pos[0]   # Motive X -> CF X
            cf_y = pos[1]   # Motive Z -> CF Y (Often inverted depending on calibration)
            cf_z = pos[2]   # Motive Y -> CF Z (Height)

            cf_qx = rot[0]
            cf_qy = rot[2]
            cf_qz = rot[1]
            cf_qw = rot[3]

            latest_pose = (cf_x, cf_y, cf_z, cf_qx, cf_qy, cf_qz, cf_qw)
            return
class AutonomousCrazyflie:
    def __init__(self, link_uri):
        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._sending_pose = False
        self._pose_thread = None
        self._is_connected = False # New flag to track connection status

        self._cf.open_link(link_uri)
        print(f'Connecting to {link_uri}')

    def _connected(self, link_uri):
        """
        Called when link is established. 
        We ONLY start the pose thread here. We do NOT set params here anymore.
        """
        print('Connected to Crazyflie!')
        self._is_connected = True # Signal the main thread that we are ready

        # Start the Position Sender Thread immediately so data starts flowing
        self._sending_pose = True
        self._pose_thread = threading.Thread(target=self._send_pose_loop)
        self._pose_thread.start()
        self._cf.platform.send_arming_request(True)

        # Setup logging
        self._setup_logging()

    def _setup_logging(self):
        lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        lg_stab.add_variable('kalman.stateX', 'float')
        lg_stab.add_variable('kalman.stateY', 'float')
        lg_stab.add_variable('kalman.stateZ', 'float')
        self._cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(self._log_callback)
        lg_stab.start()

    def _log_callback(self, timestamp, data, logconf):
        print(f"Kalman State: X={data['kalman.stateX']:.3f}, Y={data['kalman.stateY']:.3f}, Z={data['kalman.stateZ']:.3f}", end="\r")

    def _send_pose_loop(self):
        while self._sending_pose:
            if latest_pose:
                x, y, z, qx, qy, qz, qw = latest_pose
                self._cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
            time.sleep(0.05) 

    def _connection_failed(self, link_uri, msg):
        print(f'Connection failed: {msg}')

    def _connection_lost(self, link_uri, msg):
        print(f'Connection lost: {msg}')

    def _disconnected(self, link_uri):
        print('Disconnected')
        self._sending_pose = False 
        self._is_connected = False

    def run_mission(self):
        """ The main flight logic sequence """
        
        # 1. WAIT FOR CONNECTION
        print("Waiting for connection...")
        while not self._is_connected:
            time.sleep(1)

        # 2. WAIT FOR MOCAP DATA
        print("Waiting for valid Mocap data stream...")
        while latest_pose is None:
            time.sleep(0.5)

        # 3. CONFIGURE DRONE
        print("Configuring EKF parameters...")
        self._cf.param.set_value('stabilizer.estimator', '2') # EKF
        self._cf.param.set_value('locSrv.extPosStdDev', 0.01)
       

        # 3. Reset the EKF so it snaps to the new Motive coordinates
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')

        # Reset EKF
        #self._cf.param.set_value('kalman.resetEstimation', '1')
        #time.sleep(0.1)
        #self._cf.param.set_value('kalman.resetEstimation', '0')
        #time.sleep(1.0)

        # --- CRITICAL FIX: ARMING & UNLOCKING ---
        
        # 1. Send Arming Request (Software Arming)
        #self._cf.platform.send_arming_request(True)
        time.sleep(0.1)

        # 2. Unlock the Controller (The Safety Latch)
        # We send 10 packets of "0 thrust" to wake up the controller
        print("Unlocking controller...")
        for _ in range(20):
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

        try:
            # --- TAKEOFF ---
            print("Taking off...")
            # target_height=0.5m, duration=2.0s
            self._cf.high_level_commander.takeoff(0.5, 2.0) 
            time.sleep(3.0) 

            print("Hovering...")
            time.sleep(3.0)

            print("Landing...")
            self._cf.high_level_commander.land(0.0, 2.0)
            time.sleep(2.0)

        except KeyboardInterrupt:
            print("Emergency Interrupt!")
        
        finally:
            print("Stopping motors...")
            self._cf.high_level_commander.stop()
            self.disconnect()

    def disconnect(self):
        self._cf.close_link()
        self._sending_pose = False
        if self._pose_thread:
            self._pose_thread.join()

if __name__ == '__main__':
    # 1. Initialize Drivers
    cflib.crtp.init_drivers()

    

    
    
    # 2. Start Motive Client
    print("Starting Motive Client...")
    client = NatNetClient(server_ip_address=MOTIVE_IP, local_ip_address=MOTIVE_IP, use_multicast=False)
    client.connect()
    client.on_data_frame_received_event.handlers.append(receive_motive_data)
    client.run_async()
    
    # 3. Initialize Drone Class
    drone = AutonomousCrazyflie(URI)
    
    # 4. Run the mission (Blocks until done)
    # Note: The connection happens in __init__, but run_mission waits for data
    drone.run_mission()