import threading
import queue
import time
from dataclasses import dataclass
from enum import Enum, auto
from scipy.interpolate import CubicSpline
import numpy as np

class DroneCmd(Enum):
    INITIALIZE = auto()
    TAKEOFF = auto()
    LAND = auto()
    GOTO = auto()
    STEPTEST = auto()
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
    relative: bool = False
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
        self.interrupt_hover = False
        
        # --- TRAJECTORY TRACKING ---
        self.traj_splines = None
        self.traj_start_time = 0.0
        self.traj_duration = 0.0

    def run(self, cf_manager):
        """The main 50Hz control loop"""
        while self.is_running:
            loop_start = time.time()
            
            # 1. Instant Kill Switch Override
            if self.kill:
                cf_manager.cf.high_level_commander.stop() # Fixed self.cf_manager -> cf_manager
                print("Emergency Stop Activated! Motors killed.")
                self.is_running = False
                break # Safely exits the thread loop

            # 2. Hardware Interrupt Catcher (prevent thread race conditions)
            if getattr(self, 'interrupt_hover', False):
                print("Interrupt recieved! Halting current Maneuver.")
                cf_manager.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 0.0, 0.1, relative=True) # Stop drone in place and hover
                self.mode = "IDLE"                        # reset state machine
                self.interrupt_hover = False              # clear flag
            
            # 3. Process one-off commands from the GUI
            self._check_queue(cf_manager)

            # 4. Execute logic based on the current State
            self._update_state_machine(cf_manager)

            # 5. Maintain a strict 50Hz loop (20ms)
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
                        cmd.x, cmd.y, cmd.z, cmd.yaw, cmd.duration, relative=cmd.relative, linear=cmd.linear
                    )
                    self.mode = "HL_BUSY"
                    self._hl_end_time = time.time() + cmd.duration
                    
                elif cmd.action == DroneCmd.HOVER:
                    cf_manager.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 0.0, cmd.duration, relative=True)
                    self.mode = "HL_BUSY"
                    self._hl_end_time = time.time() + cmd.duration
                    
                elif cmd.action == DroneCmd.TRAJECTORY:
                    # Convert to numpy array for easy slicing
                    wp_array = np.array(cmd.waypoints)
                    
                    # Extract the time array
                    if wp_array.shape[1] == 5:
                        t = wp_array[:, 4]
                        t = t - t[0]  # Normalize so time array starts exactly at 0
                    else:
                        t = np.linspace(0, cmd.duration, len(wp_array))
                    
                    # 1. FIX THE WIGGLE: Strip out duplicate overlapping points from segment joints
                    # np.unique with return_index keeps only strictly increasing time values
                    t_unique, indices = np.unique(t, return_index=True)
                    wp_unique = wp_array[indices]
                        
                    # Build cubic splines for each axis against time
                    self.traj_splines = {
                        'x': CubicSpline(t_unique, wp_unique[:, 0], bc_type='clamped'),
                        'y': CubicSpline(t_unique, wp_unique[:, 1], bc_type='clamped'),
                        'z': CubicSpline(t_unique, wp_unique[:, 2], bc_type='clamped'),
                        'yaw': CubicSpline(t_unique, wp_unique[:, 3], bc_type='clamped')
                    }
                    
                    self.mode = "TRAJECTORY"
                    self.traj_duration = cmd.duration
                    self.traj_start_time = time.time()
                    self._hl_end_time = self.traj_start_time + cmd.duration
                    
                elif cmd.action == DroneCmd.STEPTEST:
                    #cf_manager.cf.high_level_commander.go_to(
                    #    cmd.x, cmd.y, cmd.z, cmd.yaw, 0.1, relative=False, linear=False)
                    for i in range(50):
                        cf_manager.cf.commander.send_position_setpoint( 
                            cmd.x, cmd.y, cmd.z, cmd.yaw
                        )
                        time.sleep(0.02) # 50Hz setpoint stream for 1 second
                    
                    # No state change, just a one-off command
                    
                    # Prevent watchdog crash after step test finishes
                    cf_manager.cf.commander.send_notify_setpoint_stop()
                    cf_manager.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 0.0, 0.1, relative=True)
            
            except queue.Empty:
                pass # Queue is empty, just keep flying

    def _update_state_machine(self, cf_manager):
        """Handles continuous tasks based on the active mode"""
        
        now = time.time()
        
        if self.mode == "HL_BUSY":
            if now >= self._hl_end_time:
                self.mode = "IDLE"
                print("High-level maneuver complete.")
        
        elif self.mode == "TRAJECTORY": # TRAJECTORY MODE
            
            t_elapsed = now - self.traj_start_time
            
            # Sample the continuous spline as long as we are within the duration
            if t_elapsed <= self.traj_duration and self.traj_splines is not None:
                sx = float(self.traj_splines['x'](t_elapsed))
                sy = float(self.traj_splines['y'](t_elapsed))
                sz = float(self.traj_splines['z'](t_elapsed))
                syaw = float(self.traj_splines['yaw'](t_elapsed))
                
                # Blast the interpolated setpoint continuously at loop frequency
                #cf_manager.cf.commander.send_position_setpoint(sx, sy, sz, syaw)
                
                 # Command the linear segment
                cf_manager.cf.high_level_commander.go_to(
                    sx, sy, sz, syaw, 
                    0.02, relative=False, linear=True
                )
                
                # Low Level setpoint command
                # Should be more efficient, not generating a new trajectory onboard repeatedly
                #cf_manager.cf.commander.send_position_setpoint( 
                #    wp[0], wp[1], wp[2], wp[3]
                #)

            # Once the total duration has passed, return to IDLE
            if now >= self._hl_end_time:
                # 1. Notify the firmware watchdog that the low-level stream is intentionally stopping
                cf_manager.cf.commander.send_notify_setpoint_stop()
                
                # 2. Command the onboard high-level commander to take over and hold the current position
                cf_manager.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 0.0, 0.5, relative=True)
                
                self.mode = "IDLE"
                print("Trajectory execution complete. Watchdog reset, hovering in place.")
                
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
    
    # Fixed height default to 0.0
    def land(self, height=0.0, duration=2.0):
        self.command_queue.put(DroneCommand(action=DroneCmd.LAND, height=height, duration=duration))
        
    def goto(self, x, y, z, yaw=0.0, duration=2.0, linear=False, relative=False):
        self.command_queue.put(DroneCommand(action=DroneCmd.GOTO, x=x, y=y, z=z, yaw=yaw, duration=duration, linear=linear, relative=relative))
        
    def kill_motors(self):
        self.kill = True
        
    def execute_trajectory(self, waypoints, total_duration):
        """Helper to push the trajectory into the queue"""
        self.stop_and_hover() # Clear out any existing commands to ensure a clean trajectory start
        
        if not waypoints: return # Guard against empty trajectories
        
        # 2. Transition to start of trajectory playbook
        start_wp = waypoints[0]
        transition_duration = 2.0 # Fixed transition duration for smoothness
        self.goto(
            x=start_wp[0], 
            y=start_wp[1],
            z=start_wp[2],
            yaw=start_wp[3],
            duration = transition_duration
        )
        self.command_queue.put(DroneCommand(
            action=DroneCmd.TRAJECTORY, 
            waypoints=waypoints, 
            duration=total_duration
        ))   
        #NOTE: not the spot for this maybe?
        print(f"Transitioning to start and beginning playbook execution. Total waypoints: {len(waypoints)}, Total duration: {total_duration}s")
        
    
    def step_test(self, target_position):
        self.command_queue.put(DroneCommand(action=DroneCmd.STEPTEST, x=target_position[0], y=target_position[1], z=target_position[2], yaw=0.0, duration=2.0))
        
        
        
        
        

#TODO: Class isnt used right now, GUI handles dispacting commands. 
# This might be an unnecessary class, but ill leave it incase trajectories are tricky in the gui. 
class SwarmCommandManager:
    def __init__(self, swarm_dict):
        self.swarm_dict = swarm_dict
    
    
    def swarm_takeoff(self, height, duration): 
        for agent in self.swarm_dict.values(): 
            agent.command_queue.takeoff(height, duration)
            
    def swarm_land(self, height, duration):
        for agent in self.swarm_dict.values(): 
            agent.command_queue.land(height, duration)
    
    def swarm_kill(self):
        for agent in self.swarm_dict.values(): 
            agent.command_queue.kill_motors()
      
      
    # TODO: Fix ts, goto is not in relative coordinates, all uavs will try to go to one spot      
    '''def swarm_goto(self, x, y, z, yaw, duration, linear=False):
        for agent in self.swarm_dict.values(): 
            agent.command_queue.goto(x, y, z, yaw, duration, linear)'''
            
    def swarm_trajectory(self, waypoint_dict, total_duration):
        # Waypoint dict is a dictionary mapping agent_id to a list of waypoints [(x,y,z,yaw), ...]
        for agent in self.swarm_dict.values(): 
            waypoints = waypoint_dict[agent.agent_id] # Extract the specific waypoints for this agent
            agent.command_queue.execute_trajectory(waypoints, total_duration)