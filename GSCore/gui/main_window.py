import dearpygui.dearpygui as dpg
from collections import deque
import math, time, threading
import queue
import os, glob, json
import numpy as np
from scipy.spatial.transform import Rotation

from GSCore.core.commands import DroneCmd, DroneCommand
from GSCore.core.trajectory_math import TrajectoryGenerator
from GSCore.data.logger import FlightLogger
from common_classes import SystemState


class QuasarGUI:
    def __init__(self, swarm_dict):
        self.swarm_dict = swarm_dict
        self.max_points = 500
        
        #
        
        # Swarm Formation State NOTE: I think these are good
        self.swarm_radius = 0.05 # starting radius in meters
        self.swarm_z = 0.7      # starting altitude in meters
        self.swarm_yaw = 0.0    # starting yaw in degrees
         
        #NOTE: Inject { "cf_1": queue, "cf_2": queue } here
        self.swarm_queues = {} # This should be populated with the individual CommandQueues for each drone in the swarm, keyed by drone ID. Example: {"drone_1": CommandQueue(), "drone_2": CommandQueue(), ...}
        
        # Flight logger now takes the whole swarm dictionary
        self.logger = FlightLogger(self.swarm_dict) 
        
        # Plot configurations
        self.plot_config = {
            "Position (m)": {
                "Estimate": ['ex', 'ey', 'ez'],
                "Ground Truth": ['mx', 'my', 'mz']
            },
            "Dynamics": {
                "Velocity (m/s)": ['vx', 'vy', 'vz'],
                "Acceleration (m/s2)": ['ax', 'ay', 'az'],
                "Angular Rates (rad/s)": ['p', 'q', 'r']
            },
            "Motor Commands": {
                "Motors": ['m1', 'm2', 'm3', 'm4']}
        }
        
        # Flat tracked variable list
        self.tracked_vars = []
        for groups in self.plot_config.values():
            for var_list in groups.values():
                self.tracked_vars.extend(var_list)
                
        # Generate nested buffers for EACH agent to prevent data loss when switching views
        self.history = {
            a_id: {var: deque(maxlen=self.max_points) for var in self.tracked_vars} 
            for a_id in self.swarm_dict.keys()
        }
        self.time_data = {a_id: deque(maxlen=self.max_points) for a_id in self.swarm_dict.keys()}
        self.last_t = {a_id: 0.0 for a_id in self.swarm_dict.keys()}
        
        # Trajectory Threading Variables
        self.trajectory_threads = []
        self.cancel_trajectory = False
        
        # Camera State for 3D View
        self.cam_yaw = math.pi / 4       # 45 degrees default
        self.cam_pitch = math.pi / 3     # 30 degrees default
        self.cam_scale = 80              # Pixels per meter (Zoom)
        self.cam_pan_x = 400             # Center X
        self.cam_pan_y = 350             # Center Y
        
    def get_trackball_lines(self, quat, axis_length=0.4):
            """Updated to sync perfectly with the camera's yaw, pitch, and zoom."""
            rot = Rotation.from_quat(quat)
            rot_matrix = rot.as_matrix()
            
            # Define 3D unit axes (scaled to physical length in meters)
            axes = np.array([
                [axis_length, 0, 0], # X (Red - Forward)
                [0, axis_length, 0], # Y (Green - Left)
                [0, 0, axis_length]  # Z (Blue - Up)
            ])
            rotated_axes = axes @ rot_matrix.T
            
            # Project the origin (0,0,0) to figure out the screen center point
            ox, oy = self.project_3d_to_2d(0, 0, 0)
            
            lines_2d = []
            for axis in rotated_axes:
                # Project the rotated endpoint through the camera
                px, py = self.project_3d_to_2d(axis[0], axis[1], axis[2])
                
                # Since the DPG node handles the drone's position on screen, 
                # we just return the distance from the origin.
                lines_2d.append([px - ox, py - oy])
                
            return lines_2d
    def project_3d_to_2d(self, x, y, z):
        """Corrected Dynamic 3D to 2D projection."""
        # 1. Rotate around Z-axis (Yaw)
        x1 = x * math.cos(self.cam_yaw) - y * math.sin(self.cam_yaw)
        y1 = x * math.sin(self.cam_yaw) + y * math.cos(self.cam_yaw)

        # 2. Rotate around camera's X-axis (Pitch)
        # pitch=0 -> Side view (looking at Z). pitch=pi/2 -> Top-down (looking at Y)
        screen_y_3d = y1 * math.sin(self.cam_pitch) + z * math.cos(self.cam_pitch)

        # 3. Map to screen space 
        # DPG Canvas Y goes DOWN. So to make +Z go UP, we subtract from center_y.
        screen_x = self.cam_pan_x + x1 * self.cam_scale
        screen_y = self.cam_pan_y - screen_y_3d * self.cam_scale
        
        return screen_x, screen_y
        
    def _on_mouse_drag(self, sender, app_data):
        # app_data contains [button, x_delta, y_delta]
        _, dx, dy = app_data
        
        # Only rotate if viewing the SWARM tab (optional check)
        if dpg.get_value("combo_agent_select") == "SWARM":
            # Left Click: Orbit (Yaw and Pitch)
            if dpg.is_mouse_button_down(dpg.mvMouseButton_Left):
                self.cam_yaw += dx * 0.001
                self.cam_pitch += dy * 0.001
                
                # Clamp pitch to prevent flipping upside down
                self.cam_pitch = max(0, min(self.cam_pitch, math.pi / 2.1))
                
            # Right Click: Pan (Move X/Y Center)
            elif dpg.is_mouse_button_down(dpg.mvMouseButton_Right):
                self.cam_pan_x += dx * 0.1
                self.cam_pan_y += dy * 0.1

    def _on_mouse_wheel(self, sender, app_data):
        # app_data is the scroll wheel delta (usually 1 or -1)
        if dpg.get_value("combo_agent_select") == "SWARM":
            # Zoom in/out by adjusting the scale
            self.cam_scale += app_data * 5
            self.cam_scale = max(10, min(self.cam_scale, 300)) # Limit zoom

    def _toggle_visibility(self, sender, app_data, user_data):
        is_checked = app_data
        series_tags = user_data 
        for tag in series_tags:
            if dpg.does_item_exist(tag): 
                dpg.configure_item(tag, show=is_checked)
                
    def _on_agent_selected(self, sender, app_data, user_data):
        """Swaps the UI Layout between SWARM and SINGLE view."""
        selected_view = app_data
        if selected_view == "SWARM":
            dpg.configure_item("group_single_view", show=False)
            dpg.configure_item("group_swarm_view", show=True)
            # Reset UI Flags to a neutral state
            dpg.set_value("status_vbat", "VBAT: SWARM")
            dpg.configure_item("status_vbat", color=(200, 200, 200))
            dpg.set_value("status_armed", "ARMED: MULTIPLE")
        else:
            dpg.configure_item("group_swarm_view", show=False)
            dpg.configure_item("group_single_view", show=True)

    # ==========================================
    # GUI Button Callbacks for Commands
    # ==========================================
    def _get_target_agents(self):
        """Helper to return a list of agents targeted by current UI selection."""
        target = dpg.get_value("combo_agent_select")
        if target == "SWARM":
            return list(self.swarm_dict.values())
        return [self.swarm_dict[target]]

    def cb_takeoff(self):
        for agent in self._get_target_agents():
            # 1. Grab the latest telemetry snapshot
            snap = agent.state.get_snapshot()
            
            # 2. SAFETY INTERLOCK: Check if already airborne
            if snap.get('flying', False):
                print("SAFETY: Takeoff ignored. UAV is already in the air!")
                
                # Make the UI pop up a warning text
                if dpg.does_item_exist("status_armed"):
                    dpg.configure_item("status_armed", color=(255, 150, 0)) # Flash orange
                return
            
            agent.command_queue.takeoff(height=1.0, duration=1.0)
            
    def cb_land(self):
        for agent in self._get_target_agents():
            agent.command_queue.land(height=0.0, duration=1.0)
            
    def cb_stop_hover(self):
        # Kill trajectory and hover in place
        for agent in self._get_target_agents():
            agent.command_queue.stop_and_hover()
            print("STOP HOVER: Trajectory aborted, hovering at current position.")
            
    def cb_box_bouncer(self):
        """Spins up a background thread for the reactive box-limit logic."""
        target_agents = self._get_target_agents()
        if not target_agents: return
        
        print("Starting Reactive Box Bouncer...")
        # Use an Event flag so we can kill the thread with the Emergency Stop button
        self.bouncing_active = True 
        threading.Thread(target=self._box_bounce_loop, args=(target_agents,), daemon=True).start()

    def _box_bounce_loop(self, agents):
        """The background thread that replicates move_box_limit."""
        box_limit = 0.4 # 0.5 meters
        max_vel = 0.5   # m/s
        
        while self.bouncing_active and not self.cancel_trajectory:
            for agent in agents:
                # 1. Read the thread-safe position estimate from QUASAR's state manager
                pos_x, pos_y, pos_z = agent.state.get_position()
                
                # Default velocities
                vx = 0.25
                vy = 0.25
                vz = 0.25
                
                # 2. Apply Box limits (Reactive Logic)
                if pos_x > box_limit:
                    vx = -max_vel
                elif pos_x < -box_limit:
                    vx = max_vel
                    
                if pos_y > box_limit:
                    vy = -max_vel
                elif pos_y < -box_limit:
                    vy = max_vel
                    
                if pos_z > 1.3:
                    vz = -max_vel
                elif pos_z < 0.3: # Don't let it crash into the floor
                    vz = max_vel

                # 3. Convert the vertical speed request into a hover altitude command.
                target_altitude = pos_z + vz * 0.02
                target_altitude = max(0.3, min(target_altitude, 1.3))

                # send_hover_setpoint(vx, vy, yaw_rate, z_distance)
                agent.driver.cf.commander.send_hover_setpoint(vx, vy, 0.0, target_altitude)
            
            # Sleep 100ms to mimic the time.sleep(0.1) from the original script
            time.sleep(0.02)
            
        print("Box Bouncer Terminated.")
            
    def cb_emergency_stop(self):
        # ALWAYS kill everything in the swarm, regardless of dropdown selection
        self.cancel_trajectory = True
        
        # FIXED: Loop through all agents in the swarm dictionary
        for a_id, agent in self.swarm_dict.items():
            if hasattr(agent, 'command_queue'):
                agent.command_queue.emergency_stop()
                
        print("EMERGENCY STOP: Flight Aborted and Motors Deactivated.")
        
    def cb_reload_trajectories(self):
        """Scans the trajectories folder using an absolute path relative to the project root."""
        # 1. Get the directory where THIS script (main_window.py) is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 2. Go up two levels (from GSCore/gui/ to the root QUASAR/ folder)
        project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
        
        # 3. Join with the trajectories folder name
        traj_dir = os.path.join(project_root, "trajectories")
        
        # 4. Failsafe: create the directory if it doesn't exist
        os.makedirs(traj_dir, exist_ok=True) 
        
        # 5. Look for .json files inside that specific absolute path
        file_paths = glob.glob(os.path.join(traj_dir, "*.json"))
        file_names = [os.path.basename(f) for f in file_paths]
        
        # 6. Update the DearPyGui Dropdown
        if dpg.does_item_exist("input_traj_file"):
            dpg.configure_item("input_traj_file", items=file_names)
            if file_names:
                dpg.set_value("input_traj_file", file_names[0])
            else:
                dpg.set_value("input_traj_file", "") # Clear if folder is empty
                
        print(f"Playbook scan complete. Folder: {traj_dir}")
        print(f"Found {len(file_names)} files: {file_names}")
    
    def cb_nudge_swarm(self, axis, amount):
        """Updates the formation state and physically moves the hovering drones."""
        # 1. Update the internal state
        if axis == "z":
            self.swarm_z += amount
            self.swarm_z = max(0.2, self.swarm_z) # Hard deck limit: Don't crash into the floor
        elif axis == "scale":
            self.swarm_radius += amount
            self.swarm_radius = max(0.4, self.swarm_radius) # Minimum safe distance limit
        elif axis == "yaw":
            self.swarm_yaw = (self.swarm_yaw + amount) % 360
            
        print(f"Formation Update -> Radius: {self.swarm_radius:.1f}m, Z: {self.swarm_z:.1f}m, Yaw: {self.swarm_yaw:.1f}deg")
        
        # 2. Get the Swarm Center
        center = dpg.get_value("input_swarm_center")
        cx, cy = center[0], center[1]

        # 3. Calculate new positions for the active drones using swarm_dict
        active_drones = list(self.swarm_dict.keys())
        num_drones = len(active_drones)
        
        if num_drones == 0: return

        phase_separation = 360.0 / num_drones

        for index, drone_id in enumerate(active_drones):
            agent = self.swarm_dict[drone_id]

            # Calculate this drone's angle on the circle
            '''drone_angle_deg = (index * phase_separation) + self.swarm_yaw
            theta = math.radians(drone_angle_deg)
            
            # Geometry: X = Center + (Radius * Cos), Y = Center + (Radius * Sin)
            target_x = cx + (self.swarm_radius * math.cos(theta))
            target_y = cy + (self.swarm_radius * math.sin(theta))
            target_z = self.swarm_z
            
            # Point the nose of the drone tangent to the circle (optional)
            target_yaw = drone_angle_deg + 90.0'''
            
            # 4. Dispatch the GOTO command!
            # Instead of a trajectory, we just send a 1.5s straight-line move
            if hasattr(agent, 'command_queue'):
                agent.command_queue.goto(
                    x=0, 
                    y=0, 
                    z=amount, 
                    yaw=0, 
                    duration=1.5,
                    relative=True
                )

    def cb_step_test(self, length=0.1):
        """ Shift selected UAV +0.3m in X, then back after some time, to test the one-off command functionality."""
        for agent in self._get_target_agents():
            ex, ey, ez = agent.state.get_position()
            agent.command_queue.step_test((ex + length, ey, ez)) # Move +0.3m in X
        time.sleep(3) # Wait for 3 seconds
        for agent in self._get_target_agents():
            ex, ey, ez = agent.state.get_position()
            agent.command_queue.step_test((ex - length, ey, ez)) # Move back to original position
        
    def cb_manual_goto(self):
        """One-off point jump using the goto() method in commands.py."""
        x, y, z = dpg.get_value("input_man_pos")
        yaw = dpg.get_value("input_man_yaw")
        dur = dpg.get_value("input_man_dur")
        
        # Dispatch to selected agents
        for agent in self._get_target_agents():
            agent.command_queue.goto(x, y, z, yaw=yaw, duration=dur)

    def cb_goto(self):
        """Reads the selected JSON, applies real-world spatial offset (X, Y, and Z) & rotation, and executes."""
        target_agents = self._get_target_agents()
        if not target_agents: return
        
        file_name = dpg.get_value("input_traj_file")
        if not file_name:
            print("ERROR: No trajectory selected!")
            return
            
        # Robust pathing to the project root
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
        filepath = os.path.join(project_root, "trajectories", file_name)
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                
            raw_waypoints = data["waypoints"]
            duration = data["total_duration"]
            
            if not raw_waypoints:
                print("ERROR: Trajectory file contains no waypoints.")
                return

            # ==========================================
            # FIXED-ORIGIN SWARM ROTATION & PLACEMENT
            # ==========================================
            
            # 1. Get the Mission Center from the GUI input box
            mission_center = dpg.get_value("input_swarm_center")
            cx = mission_center[0]
            cy = mission_center[1]
            cz = mission_center[2]
            
            # 2. Get the Swarm Rotation from your live D-Pad state
            theta = math.radians(self.swarm_yaw)
            
            translated_waypoints = []
            
            for wp in raw_waypoints:
                # A. Rotate the raw playbook X and Y around (0,0)
                rot_x = (wp[0] * math.cos(theta)) - (wp[1] * math.sin(theta))
                rot_y = (wp[0] * math.sin(theta)) + (wp[1] * math.cos(theta))
                
                # B. Translate (Shift) the rotated shape to the chosen Mission Center
                new_x = rot_x + cx
                new_y = rot_y + cy
                
                # C. Apply the Altitude (Playbook Z + Center Z)
                new_z = wp[2] + cz 
                
                # D. Rotate the drone's nose
                designed_yaw = wp[3] if len(wp) > 3 else 0.0
                new_yaw = designed_yaw + self.swarm_yaw
                
                # Format for CommandQueue: (X, Y, Z, Yaw)
                translated_waypoints.append((new_x, new_y, new_z, new_yaw))
                
            # ==========================================
            
            # Hand off the completely calculated absolute path to the Drone's Queue
            for agent in target_agents:
                agent.command_queue.execute_trajectory(translated_waypoints, duration)
            
            print(f"Executing playbook: {data.get('name', file_name)}")
            print(f"Mission Center -> X: {cx:.2f}m, Y: {cy:.2f}m, Z: {cz:.2f}m | Swarm Yaw: {self.swarm_yaw:.1f} deg")    
        except Exception as e:
            import traceback
            print(f"Failed to load trajectory file: {e}")
            traceback.print_exc()
        
        
    def cb_log(self):
        self.logger.toggle_logging() 
        if self.logger.is_logging:
            dpg.configure_item("btn_record", label="STOP RECORDING")
            dpg.bind_item_theme("btn_record", "theme_recording_active")
        else:
            dpg.configure_item("btn_record", label="START RECORDING")
            dpg.bind_item_theme("btn_record", "theme_recording_idle")
    
    def setup_gui(self):
        dpg.create_context()
        
        with dpg.theme(tag="theme_recording_idle"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (100, 100, 100))
                
        with dpg.theme(tag="theme_recording_active"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40))
        
        with dpg.window(label="QUASAR Telemetry", width=1200, height=800):
            
            # --- THE MULTI-AGENT DROPDOWN ---
            agent_options = ["SWARM"] + list(self.swarm_dict.keys())
            with dpg.group(horizontal=True):
                dpg.add_text("Active View:", color=(255, 200, 50))
                dpg.add_combo(items=agent_options, default_value="SWARM", tag="combo_agent_select", callback=self._on_agent_selected, width=150)
                
            dpg.add_separator()
            
            # ==========================================
            # STATUS PANEL
            # ==========================================
            with dpg.group(horizontal=True):
                dpg.add_text("STATUS |", color=(200, 200, 200))
                dpg.add_text("VBAT: -- V", tag="status_vbat")
                dpg.add_text("|")
                dpg.add_text("ARMED: FALSE", tag="status_armed", color=(150, 150, 150))
                dpg.add_text("FLYING: FALSE", tag="status_flying", color=(150, 150, 150))
                dpg.add_text("CRASHED: FALSE", tag="status_crashed", color=(150, 150, 150))
                dpg.add_text("LOCKED: FALSE", tag="status_locked", color=(150, 150, 150))
                dpg.add_button(label="START RECORDING", width=150, height=40, tag="btn_record", callback=self.cb_log)
                dpg.bind_item_theme("btn_record", "theme_recording_idle")
                dpg.add_spacer(width=20)
                dpg.add_button(label="STEP TEST", width=120, height=40, callback=self.cb_step_test)
                
            dpg.add_spacer(height=10) 
            dpg.add_separator()
            
            # ==========================================
            # COMMAND & CONTROL PANEL
            # ==========================================
            with dpg.group(horizontal=True):
                with dpg.group():
                    dpg.add_text("Flight Controls", color=(200, 200, 200))
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="TAKEOFF", width=100, height=40, callback=self.cb_takeoff)
                        dpg.add_button(label="LAND", width=100, height=40, callback=self.cb_land)
                        dpg.add_button(label="BOX BOUNCE", width=120, height=40, callback=self.cb_box_bouncer)
                        hover_button = dpg.add_button(label="HOVER", width=100, height=40, callback=self.cb_stop_hover)
                        emergency_button = dpg.add_button(label="EMERGENCY STOP", width=150, height=40, callback=self.cb_emergency_stop)
                        
                        # Bind a theme specifically to the HOVER button (yellow)
                        with dpg.theme() as hover_theme:
                            with dpg.theme_component(dpg.mvButton):
                                dpg.add_theme_color(dpg.mvThemeCol_Button, (220, 180, 0))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 220, 80))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (180, 140, 0))
                        dpg.bind_item_theme(hover_button, hover_theme)

                        # Bind a theme specifically to the EMERGENCY STOP button (red)
                        with dpg.theme() as kill_theme:
                            with dpg.theme_component(dpg.mvButton):
                                dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 50, 50))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 30, 30))
                        dpg.bind_item_theme(emergency_button, kill_theme)
                    

                dpg.add_spacer(width=30)

                # ==========================================
                # TRAJECTORY PLAYBOOK & SWARM CONTROLS
                # ==========================================
                with dpg.group():
                    dpg.add_text("Swarm Formation Controls", color=(200, 200, 200))
                    
                    # Row 1: Altitude & Scale
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="^ RAISE", width=80, callback=lambda: self.cb_nudge_swarm("z", 0.2))
                        dpg.add_button(label="v LOWER", width=80, callback=lambda: self.cb_nudge_swarm("z", -0.2))
                        dpg.add_spacer(width=10)
                        dpg.add_button(label="< > SPREAD", width=80, callback=lambda: self.cb_nudge_swarm("scale", 0.2))
                        dpg.add_button(label="> < CLUSTER", width=80, callback=lambda: self.cb_nudge_swarm("scale", -0.2))
                        
                    # Row 2: Rotation & Center
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="↺ ROTATE CCW", width=80, callback=lambda: self.cb_nudge_swarm("yaw", 15.0))
                        dpg.add_button(label="↻ ROTATE CW", width=80, callback=lambda: self.cb_nudge_swarm("yaw", -15.0))
                        dpg.add_spacer(width=10)
                        dpg.add_input_floatx(size=3,label="Center (X,Y,Z)", tag="input_swarm_center", default_value=[0.0, 0.0, 0.0], width=150)

                    dpg.add_spacer(height=10)
                    
                    # Row 3: Playbook Execution
                    dpg.add_text("Mission Execution", color=(200, 200, 200))
                    with dpg.group(horizontal=True):
                        dpg.add_combo(items=[], tag="input_traj_file", width=150)
                        dpg.add_button(label="RELOAD", callback=self.cb_reload_trajectories)
                    
                    dpg.add_button(label="EXECUTE TRAJECTORY", tag="btn_execute", width=-1, height=35, callback=self.cb_goto)
                    with dpg.theme() as exec_theme:
                        with dpg.theme_component(dpg.mvButton):
                            dpg.add_theme_color(dpg.mvThemeCol_Button, (40, 150, 40)) # Greenish
                            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (50, 200, 50))
                            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (30, 120, 30))
                    dpg.bind_item_theme("btn_execute", exec_theme)
                    
            dpg.add_separator()
            dpg.add_spacer(height=10)
            
            # ==========================================
            # LAYOUT 1: SWARM OVERVIEW (Default)
            # ==========================================
            with dpg.group(tag="group_swarm_view", show=True):
                    dpg.add_text("3D Swarm Spatial Overview", color=(150, 150, 255))
                    
                    # Create a blank drawing canvas
                    with dpg.drawlist(width=800, height=500, tag="swarm_3d_canvas"):
                        # Draw a dark background
                        dpg.draw_rectangle([0,0], [800,500], color=[50,50,50], fill=[30,30,30])
                        
                            # --- NEW: Generate tags for the grid lines ---
                        self.grid_size = 3 # -3m to +3m
                        self.grid_lines = []
                        
                        # X-axis parallel lines
                        for y in range(-self.grid_size, self.grid_size + 1):
                            tag = f"grid_x_{y}"
                            dpg.draw_line([0,0], [0,0], color=[80, 80, 80, 150], thickness=1, tag=tag)
                            self.grid_lines.append(('x', y, tag))
                            
                        # Y-axis parallel lines
                        for x in range(-self.grid_size, self.grid_size + 1):
                            tag = f"grid_y_{x}"
                            dpg.draw_line([0,0], [0,0], color=[80, 80, 80, 150], thickness=1, tag=tag)
                            self.grid_lines.append(('y', x, tag))
                        
                        # Create a trackball node for each drone
                        for a_id in self.swarm_dict.keys():
                            with dpg.draw_node(tag=f"swarm_node_{a_id}"):
                                # Draw the X, Y, Z orientation lines
                                dpg.draw_line([0,0], [0,0], color=[255, 50, 50], thickness=2, tag=f"s_tb_x_{a_id}")
                                dpg.draw_line([0,0], [0,0], color=[50, 255, 50], thickness=2, tag=f"s_tb_y_{a_id}")
                                dpg.draw_line([0,0], [0,0], color=[50, 150, 255], thickness=2, tag=f"s_tb_z_{a_id}")
                                
                                # Add a label hovering slightly above the drone
                                dpg.draw_text([10, -20], text=a_id, color=[200, 200, 200], size=15)


            # ==========================================
            # LAYOUT 2: SINGLE AGENT DETAILS (Hidden by default)
            # ==========================================
            with dpg.group(tag="group_single_view", show=False):
                with dpg.group(): 
                    dpg.add_text("Toggle Variable Groups:")
                    for chart_name, groups in self.plot_config.items():
                        with dpg.group(horizontal=True):
                            dpg.add_text(f"{chart_name}:", color=(200, 200, 200))
                            for group_name, variables in groups.items():
                                tags = [f"series_{var}" for var in variables]
                                dpg.add_checkbox(label=group_name, default_value=True, callback=self._toggle_visibility, user_data=tags)

                with dpg.group(horizontal=True):
                    with dpg.group(width=-180):
                        num_charts = len(self.plot_config)
                        with dpg.subplots(num_charts, 1, label="Telemetry", width=-1, height=-1, link_all_x=True):
                            for chart_name, groups in self.plot_config.items():
                                with dpg.plot(label=chart_name):
                                    dpg.add_plot_legend()
                                    x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
                                    if not hasattr(self, 'master_x_axis'):
                                        self.master_x_axis = x_axis 
                                        
                                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=chart_name)
                                    dpg.set_axis_limits_auto(y_axis)
                                    for group_name, variables in groups.items():
                                        for var in variables:
                                            dpg.add_line_series([], [], label=f"{group_name} {var.upper()}", tag=f"series_{var}", parent=y_axis)

                    with dpg.group():
                        dpg.add_text("Orientation", color=(200, 200, 200))
                        with dpg.drawlist(width=150, height=150):
                            dpg.draw_rectangle([0,0], [150,150], color=[50,50,50], fill=[30,30,30])
                            with dpg.draw_node(tag="trackball_node"):
                                dpg.draw_line([0,0], [0,0], color=[255, 50, 50], thickness=3, tag="tb_x")
                                dpg.draw_line([0,0], [0,0], color=[50, 255, 50], thickness=3, tag="tb_y")
                                dpg.draw_line([0,0], [0,0], color=[50, 150, 255], thickness=3, tag="tb_z")
                        dpg.apply_transform("trackball_node", dpg.create_translation_matrix([75, 75]))

        dpg.create_viewport(title='QUASAR Swarm Testbed', width=1250, height=850)
        
        dpg.create_viewport(title='QUASAR Swarm Testbed', width=1250, height=850)
        
        # Handler registries can exist globally, so this stays
        with dpg.handler_registry():
            dpg.add_mouse_drag_handler(callback=self._on_mouse_drag)
            dpg.add_mouse_wheel_handler(callback=self._on_mouse_wheel)
            
        self.cb_reload_trajectories() 
        
        dpg.create_viewport(title='QUASAR Testbed', width=1250, height=850)

        dpg.setup_dearpygui()
        dpg.show_viewport()
        
    def run(self):
        start_time = time.time()
        view_window_seconds = 5.0 
        
        while dpg.is_dearpygui_running():
            current_view = dpg.get_value("combo_agent_select")
            
            # 1. Constantly update background buffers for ALL agents
            for a_id, agent in self.swarm_dict.items():
                snap = agent.state.get_snapshot()
                
                if snap['t'] > self.last_t[a_id]:
                    self.last_t[a_id] = snap['t']
                    current_time = time.time() - start_time
                    self.time_data[a_id].append(current_time)
                    
                    for var in self.tracked_vars:
                        self.history[a_id][var].append(snap[var])

                   
                    # 2A. Update Isometric 3D Canvas if we are in Swarm View
                    if current_view == "SWARM":
                        
                        for axis, val, tag in self.grid_lines:
                            if axis == 'x':
                                p1 = self.project_3d_to_2d(-self.grid_size, val, 0)
                                p2 = self.project_3d_to_2d(self.grid_size, val, 0)
                            else:
                                p1 = self.project_3d_to_2d(val, -self.grid_size, 0)
                                p2 = self.project_3d_to_2d(val, self.grid_size, 0)
                            dpg.configure_item(tag, p1=p1, p2=p2)
                        # 1. Project the drone's 3D position to the 2D canvas
                        sx, sy = self.project_3d_to_2d(
                            snap['mx'], snap['my'], snap['mz'], 
                            
                        )
                        
                        # 2. Translate the entire node to that pixel coordinate
                        dpg.apply_transform(
                            f"swarm_node_{a_id}", 
                            dpg.create_translation_matrix([sx, sy])
                        )
                        
                        # 3. Calculate and apply the rotated trackball lines
                        # (Scaled down to 30 so they aren't massive on the swarm map)
                        endpoints = self.get_trackball_lines(
                            [snap['mqx'], snap['mqy'], snap['mqz'], snap['mqw']], 
                            axis_length=0.4 # Lines are drawn 0.4 meters long
                        
                        )
                        
                        dpg.configure_item(f"s_tb_x_{a_id}", p2=endpoints[0])
                        dpg.configure_item(f"s_tb_y_{a_id}", p2=endpoints[1])
                        dpg.configure_item(f"s_tb_z_{a_id}", p2=endpoints[2])

            # 2B. Update detailed plots if viewing a specific agent
            if current_view != "SWARM":
                snap = self.swarm_dict[current_view].state.get_snapshot()
                x_list = list(self.time_data[current_view])
                
                # Update line series
                for var in self.tracked_vars:
                    dpg.set_value(f"series_{var}", [x_list, list(self.history[current_view][var])])
                
                # Update Trackball 
                endpoints = self.get_trackball_lines([snap['eqx'], snap['eqy'], snap['eqz'], snap['eqw']])
                dpg.configure_item("tb_x", p2=endpoints[0])
                dpg.configure_item("tb_y", p2=endpoints[1])
                dpg.configure_item("tb_z", p2=endpoints[2])
                
                # Update Status Values
                vbat = snap['vbat']
                dpg.set_value("status_vbat", f"VBAT: {vbat:.2f} V")
                if vbat < 3.3: dpg.configure_item("status_vbat", color=(255, 50, 50)) 
                elif vbat < 3.6: dpg.configure_item("status_vbat", color=(255, 200, 50)) 
                else: dpg.configure_item("status_vbat", color=(50, 255, 50)) 

                # Flag Updates
                dpg.set_value("status_armed", f"ARMED: {snap['armed']}")
                dpg.configure_item("status_armed", color=(255, 50, 50) if snap['armed'] else (150, 150, 150))
                
                dpg.set_value("status_flying", f"FLYING: {snap['flying']}")
                dpg.configure_item("status_flying", color=(50, 255, 50) if snap['flying'] else (150, 150, 150))

                dpg.set_value("status_crashed", f"CRASHED: {snap['crashed']}")
                dpg.configure_item("status_crashed", color=(255, 0, 0) if snap['crashed'] else (150, 150, 150))
                
                dpg.set_value("status_locked", f"LOCKED: {snap['locked']}")
                dpg.configure_item("status_locked", color=(255, 0, 0) if snap['locked'] else (150, 150, 150))

                # X-Axis Scrolling
                if x_list:
                    min_t = max(0.0, x_list[-1] - view_window_seconds)
                    dpg.set_axis_limits(self.master_x_axis, min_t, x_list[-1])
            
            dpg.render_dearpygui_frame()
            
        dpg.destroy_context()


def start_gui(swarm_dict):
    """Entry point from main.py"""
    gui = QuasarGUI(swarm_dict)
    gui.setup_gui()
    gui.run()