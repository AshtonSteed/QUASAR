# Might not need, cflib wrappers for common stuff
# Sending 6DOF to UAV, changing setpoint, UAV parameters, logging functions
import time
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie

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

    # ==========================================
    # THE CONTROL WORKER
    # ==========================================

    def _control_loop(self):
        """
        The main flight loop. Runs in background thread.
        """
        