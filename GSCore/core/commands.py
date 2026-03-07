import threading
import queue
import time
from dataclasses import dataclass
from enum import Enum, auto
import queue

# All high-level commands build into GUI
# ENUM gives a solid list of commands
class DroneCmd(Enum):
    INITIALIZE = auto()
    TAKEOFF = auto()
    LAND = auto()
    GOTO = auto()
    START_LL_STREAM = auto()

# Holds all info required for commands
@dataclass
class DroneCommand:
    action: DroneCmd      # This is the ONLY required field
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    height: float = 0.0
    duration: float = 0.0

class CommandQueue():
    def __init__(self):
        super().__init__()
        self.high_level_commands = queue.Queue()
        self.shared_ll_setpoint = queue.Queue() # The thread-safe target object, TODO: Figure out low level setpoints
        

        self.is_running = True
        self.mode = "IDLE"           # States: IDLE, HL_BUSY, LL_STREAMING
        self.hl_maneuver_end_time = 0.0
        self.kill = False

    # Logic for running commands
    def run(self, cf):
        """The main 50Hz control loop"""
        while self.is_running:
            loop_start = time.time()
            
            if self.kill:
                self.cf.high_level_commander.stop()
                print("Emergency Stop Activated! Motors killed.")
                self.is_running = False
                continue

            # 1. Process one-off commands from the GUI
            self._check_queue(cf)

            # 2. Execute logic based on the current State
            self._update_state_machine(cf)

            # 3. Maintain a strict 50Hz loop (20ms)
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, 0.02 - elapsed))

    def _check_queue(self, cf):
        """Non-blocking check for GUI commands"""
        while self._is_running:
            if self.mode == "IDLE":
                try:
                    # Grab the DroneCommand object from the queue
                    cmd = self._cmd_queue.get_nowait()

                    # Route using dot-notation
                    
                    if cmd.action == DroneCmd.INITIALIZE:
                        print("Resetting EKF...")
                        # Reset Kalman Filter
                        self.cf.param.set_value('kalman.resetEstimation', '1') 
                        # Tell command queue to wait for EKF to converge
                        self.mode = "INITIALIZING"
                    elif cmd.action == DroneCmd.TAKEOFF:
                        cf.high_level_commander.takeoff(cmd.height, cmd.duration)
                        self.mode = "HL_BUSY"
                        self._hl_end_time = time.time() + cmd.duration
                        
                    elif cmd.action == DroneCmd.LAND:
                        cf.high_level_commander.land(cmd.height, cmd.duration)
                        self.mode = "HL_BUSY"
                        self._hl_end_time = time.time() + cmd.duration
                        
                    elif cmd.action == DroneCmd.GOTO:
                        cf.high_level_commander.go_to(
                            cmd.x, cmd.y, cmd.z, cmd.yaw, cmd.duration, relative=False
                        )
                        self.mode = "HL_BUSY"
                        self._hl_end_time = time.time() + cmd.duration
                        

                except queue.Empty:
                    break

    def _update_state_machine(self):
        """Handles continuous tasks based on the active mode"""
        if self.mode == "HL_BUSY":
            # Just check if the maneuver is done
            if time.time() >= self.hl_maneuver_end_time:
                self.mode = "IDLE"
                print("High-level maneuver complete.")
                
        if self.mode == "INITIALIZING":
                # Check if our variances have dropped below the safe threshold
                threshold = 0.001
                if (self._variance['x'] < threshold and 
                    self._variance['y'] < threshold and 
                    self._variance['z'] < threshold):
                    
                    print("EKF Converged Successfully! Ready for flight.")
                    self.mode = "IDLE"

        elif self.mode == "LL_STREAMING":
            # Grab the latest target from the thread-safe object
            target = self.shared_ll_setpoint.get_latest_pose()
            if target:
                # Blast the setpoint to the radio at 50Hz
                self.cf.commander.send_position_setpoint(
                    target['x'], target['y'], target['z'], target['yaw']
                )

    def stop(self):
        self.is_running = False
        
    # Add Commands from GUI to command queue, will execute in order in main loop
    def takeoff(self, height=1.0, duration=2.0):
        cmd = DroneCommand(action=DroneCmd.INITIALIZE)
        self.command_queue.put(cmd)
        cmd = DroneCommand(action=DroneCmd.TAKEOFF, height=height, duration=duration)
        self.command_queue.put(cmd) 
    
    def land(self, height=1.0, duration=2.0):
        cmd = DroneCommand(action=DroneCmd.LAND, height=height, duration=duration)
        self.command_queue.put(cmd) 
        
    def goto(self, x, y, z, yaw=0.0, duration=2.0):
        cmd = DroneCommand(action=DroneCmd.GOTO, x=x, y=y, z=z, yaw=yaw, duration=duration)
        self.command_queue.put(cmd) 
        
    def kill_motors(self):
        self.kill = True