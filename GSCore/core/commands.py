import threading
import queue
import time
from dataclasses import dataclass
from enum import Enum, auto

class DroneCmd(Enum):
    INITIALIZE = auto()
    TAKEOFF = auto()
    LAND = auto()
    GOTO = auto()
    START_LL_STREAM = auto()
    HOVER = auto()
    TRAJECTORY = auto()

@dataclass
class DroneCommand:
    action: DroneCmd      
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    height: float = 0.0
    duration: float = 0.0
    linear: bool = False # For future use in trajectory following
    waypoints: list = None # For trajectory commands, list of (x,y,z,yaw) tuples

class CommandQueue: # Removed empty parenthesis
    def __init__(self):
        # Removed super().__init__()
        self.command_queue = queue.Queue()
        self.shared_ll_setpoint = None # TODO: Replace with custom PoseState object
        
        self.is_running = True
        self.mode = "IDLE"           
        self._hl_end_time = 0.0 # Fixed variable name
        self.kill = False
        
        # --- NEW TRAJECTORY TRACKING ---
        self.current_trajectory = []
        self.traj_step_index = 0
        self.traj_step_duration = 0.0
        self.traj_next_step_time = 0.0

    def run(self, cf_manager):
        """The main 50Hz control loop"""
        while self.is_running:
            loop_start = time.time()
            
            # Instant Kill Switch Override
            if self.kill:
                cf_manager.cf.high_level_commander.stop() # Fixed self.cf_manager -> cf_manager
                print("Emergency Stop Activated! Motors killed.")
                self.is_running = False
                break # Safely exits the thread loop

            # 1. Process one-off commands from the GUI
            self._check_queue(cf_manager)

            # 2. Execute logic based on the current State
            self._update_state_machine(cf_manager)

            # 3. Maintain a strict 50Hz loop (20ms)
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, 0.02 - elapsed))

    # TODO: Ensure that UAV hovers in place after ALL commands
    def _check_queue(self, cf_manager):
        """Non-blocking check for GUI commands"""
        # Removed the inner while loop. The main loop handles the spinning.
        if self.mode == "IDLE":
            try:
                cmd = self.command_queue.get_nowait()
                
                if cmd.action == DroneCmd.INITIALIZE:
                    print("Resetting EKF...")
                    cf_manager.cf.param.set_value('kalman.resetEstimation', '1') 
                    self.mode = "INITIALIZING"
                    
                elif cmd.action == DroneCmd.TAKEOFF:
                    cf_manager.cf.platform.send_arming_request(True)
                    time.sleep(0.5)
                    cf_manager.cf.high_level_commander.takeoff(cmd.height, cmd.duration, yaw=None)
                    self.mode = "HL_BUSY"
                    self._hl_end_time = time.time() + 2 * cmd.duration
                    
                elif cmd.action == DroneCmd.LAND:
                    cf_manager.cf.high_level_commander.land(cmd.height, cmd.duration)
                    self.mode = "HL_BUSY"
                    self._hl_end_time = time.time() + cmd.duration
                    
                elif cmd.action == DroneCmd.GOTO:
                    cf_manager.cf.high_level_commander.go_to(
                        cmd.x, cmd.y, cmd.z, cmd.yaw, cmd.duration, relative=False
                    )
                    self.mode = "HL_BUSY"
                    self._hl_end_time = time.time() + cmd.duration
                    
                elif cmd.action == DroneCmd.HOVER:
                    cf_manager.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 0.0, cmd.duration, relative=True)
                    self.mode = "HL_BUSY"
                    self._hl_end_time = time.time() + cmd.duration
                    
                elif cmd.action == DroneCmd.TRAJECTORY:
                    self.current_trajectory = cmd.waypoints
                    self.traj_step_index = 0
                    # Total time divided by number of points gives us segment timing
                    self.traj_step_duration = cmd.duration / len(cmd.waypoints)
                    self.mode = "TRAJECTORY"
                    self._hl_end_time = time.time() + cmd.duration
                    self.traj_next_step_time = time.time() # Start first point now  
            except queue.Empty:
                pass # Queue is empty, just keep flying

    def _update_state_machine(self, cf_manager):
        """Handles continuous tasks based on the active mode"""
        if self.mode == "HL_BUSY":
            if time.time() >= self._hl_end_time:
                self.mode = "IDLE"
                print("High-level maneuver complete.")
        
        elif self.mode == "TRAJECTORY": #NEW TRAJECTORY MODE
            now = time.time()
            
            # Check if it's time for the next waypoint
            if now >= self.traj_next_step_time and self.traj_step_index < len(self.current_trajectory):
                wp = self.current_trajectory[self.traj_step_index]
                
                # Command the linear segment
                cf_manager.cf.high_level_commander.go_to(
                    wp[0], wp[1], wp[2], wp[3], 
                    self.traj_step_duration, relative=False, linear=True
                )
                
                self.traj_step_index += 1
                self.traj_next_step_time = now + self.traj_step_duration

            # Once the total duration has passed, return to IDLE
            if now >= self._hl_end_time:
                self.mode = "IDLE"
                print("Trajectory arc complete.")
                
        elif self.mode == "INITIALIZING":
            threshold = 0.001

            if (cf_manager._variance['x'] < threshold and 
                cf_manager._variance['y'] < threshold and 
                cf_manager._variance['z'] < threshold):
                print("EKF Converged Successfully! Ready for flight.")
                self.mode = "IDLE"

        elif self.mode == "LL_STREAMING":
            if self.shared_ll_setpoint:
                target = self.shared_ll_setpoint.get_latest_pose()
                if target:
                    cf_manager.cf.commander.send_position_setpoint( # Fixed self.cf_manager -> cf_manager
                        target['x'], target['y'], target['z'], target['yaw']
                    )
                    

# GUI ACCESSIBLE COMMAND METHODS
    def stop_and_hover(self):
        """Immediately clears all pending commands and halts the drone in place."""
        # 1. Wipe out any pending actions in the queue (Thread-safe)
        with self.command_queue.mutex:
            self.command_queue.queue.clear()
        # 2. Trigger the hardware interrupt on the next 50Hz loop tick
        self.interrupt_hover = True
        
    def emergency_stop(self):
        self.kill_motors()
        
    def takeoff(self, height=1.0, duration=4.0):
        print("Takeoff command received from GUI.")
        self.command_queue.put(DroneCommand(action=DroneCmd.INITIALIZE))
        self.command_queue.put(DroneCommand(action=DroneCmd.TAKEOFF, height=height, duration=duration))
        self.command_queue.put(DroneCommand(action=DroneCmd.HOVER, duration=duration))
    
    # Fixed height default to 0.0
    def land(self, height=0.0, duration=2.0):
        self.command_queue.put(DroneCommand(action=DroneCmd.LAND, height=height, duration=duration))
        
    def goto(self, x, y, z, yaw=0.0, duration=2.0, linear=False):
        self.command_queue.put(DroneCommand(action=DroneCmd.GOTO, x=x, y=y, z=z, yaw=yaw, duration=duration, linear=linear))
        
    def kill_motors(self):
        self.kill = True
        
    def execute_trajectory(self, waypoints, total_duration):
        """Helper to push the trajectory into the queue"""
        self.stop_and_hover() # Clear out any existing commands to ensure a clean trajectory start
        
        self.command_queue.put(DroneCommand(
            action=DroneCmd.TRAJECTORY, 
            waypoints=waypoints, 
            duration=total_duration
        ))   