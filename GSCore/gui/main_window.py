# Whatever driver script is needed to display GUI 
# Probably have seperate files for, plotting telemetry, sending commands, etc.

import queue

import dearpygui.dearpygui as dpg
from collections import deque
import math, time, threading
import os, glob, json
import numpy as np
from scipy.spatial.transform import Rotation

from GSCore.core.commands import DroneCmd, DroneCommand
from GSCore.tools.trajectory_math import TrajectoryGenerator
from GSCore.drivers.mock_motive_client import start_mock_stream
from GSCore.data.logger import FlightLogger
from common_classes import SystemState


class QuasarGUI:
    def __init__(self, shared_state, command_queue=None):
        self.state = shared_state
        self.cmd_queue = command_queue
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.logger = FlightLogger(self.state)
        
        # Swarm Formation State NOTE: delete this if it fucks with the swarmCommander
        self.swarm_radius = 1.0 # starting radius in meters
        self.swarm_z = 1.0      # starting altitude in meters
        self.swarm_yaw = 0.0    # starting yaw in degrees 
        #NOTE: This probably doesn't belong here, consider moving it to the swarm commander
        self.swarm_queues = {} # This should be populated with the individual CommandQueues for each drone in the swarm, keyed by drone ID. Example: {"drone_1": CommandQueue(), "drone_2": CommandQueue(), ...}
        # auto-generated the comment above so it might be cappin
        
        # Definition of all charted variables, their groups, and plots
        # Keys = Chart Names, Values = The exact keys from your get_snapshot()
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
        # Generate buffers for each variable
        self.history = {var: deque(maxlen=self.max_points) for var in self.tracked_vars}
        
        # Trajectory Threading Variables
        self.trajectory_thread = None
        self.cancel_trajectory = False
        
    
    def get_trackball_lines(self, quat, scale=50):
        rot = Rotation.from_quat(quat)
        rot_matrix = rot.as_matrix()
        #Define our base 3D unit vectors (X: Forward, Y: Left, Z: Up)
        axes = np.array([
            [1, 0, 0], # X axis (Red)
            [0, 1, 0], # Y axis (Green)
            [0, 0, 1]  # Z axis (Blue)
        ])
        rotated_axes = axes @ rot_matrix.T
        
        # Scale them up for the GUI, set up for gui
        lines_2d = []
        for axis in rotated_axes:
            screen_x = axis[0] * scale
            screen_y = -axis[2] * scale # Mapping 3D Z (Up) to Screen -Y (Up)
            lines_2d.append([screen_x, screen_y])
            
        return lines_2d # Returns endpoints for the X, Y, and Z lines
    def _toggle_visibility(self, sender, app_data, user_data):
        """Hides or shows multiple line series based on the checkbox state"""
        is_checked = app_data
        series_tags = user_data # This is now a list of strings
        
        # Loop through every tag in the list and apply the show/hide state
        for tag in series_tags:
            # Optional failsafe: check if the item exists before configuring it
            if dpg.does_item_exist(tag): 
                dpg.configure_item(tag, show=is_checked)
                
    # ==========================================
    # GUI Button Callbacks for Commands
    # ==========================================
    def cb_takeoff(self):
        print("takeoff 1")
        if self.cmd_queue:
            print("takeoff 2")
            self.cmd_queue.takeoff(height=1.0, duration=2.0)
            
    def cb_land(self):
        if self.cmd_queue:
            self.cmd_queue.land(height=0.0, duration=2.0)
            
    def cb_stop_hover(self):
        # Kill trajectory and hover in place
        if self.cmd_queue:
            self.cmd_queue.stop_and_hover()
            print("STOP HOVER: Trajectory aborted, hovering at current position.")
            
    def cb_emergency_stop(self):
        # 1. Kill any active trajectory stream in GUI
        self.cancel_trajectory = True
        
        # 2. Send hardware halt command
        if self.cmd_queue:
            self.cmd_queue.emergency_stop()
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

        # 3. Calculate new positions for the active drones
        # Note: You will need to pull your list of active drones here. 
        # For this example, let's assume you have 3 active drones.
        active_drones = ["drone_1", "drone_2", "drone_3"] 
        num_drones = len(active_drones)
        
        if num_drones == 0: return

        phase_separation = 360.0 / num_drones

        for index, drone_id in enumerate(active_drones):
            # Calculate this drone's angle on the circle
            drone_angle_deg = (index * phase_separation) + self.swarm_yaw
            theta = math.radians(drone_angle_deg)
            
            # Geometry: X = Center + (Radius * Cos), Y = Center + (Radius * Sin)
            target_x = cx + (self.swarm_radius * math.cos(theta))
            target_y = cy + (self.swarm_radius * math.sin(theta))
            target_z = self.swarm_z
            
            # Point the nose of the drone tangent to the circle (optional)
            target_yaw = drone_angle_deg + 90.0
            
            # 4. Dispatch the GOTO command!
            # Instead of a trajectory, we just send a 1.5s straight-line move
            if drone_id in self.swarm_queues:
                self.swarm_queues[drone_id].goto(
                    x=target_x, 
                    y=target_y, 
                    z=target_z, 
                    yaw=target_yaw, 
                    duration=1.5
                )

    def cb_manual_goto(self):
        """One-off point jump using the goto() method in commands.py."""
        if self.cmd_queue:
            x, y, z = dpg.get_value("input_man_pos")
            yaw = dpg.get_value("input_man_yaw")
            dur = dpg.get_value("input_man_dur")
            self.cmd_queue.goto(x, y, z, yaw=yaw, duration=dur)

    def cb_goto(self):
        """Reads the selected JSON, applies real-world spatial offset (X, Y, and Z) & rotation, and executes."""
        if not self.cmd_queue: return
        
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
            self.cmd_queue.execute_trajectory(translated_waypoints, duration)
            
            print(f"Executing playbook: {data.get('name', file_name)}")
            print(f"Mission Center -> X: {cx:.2f}m, Y: {cy:.2f}m, Z: {cz:.2f}m | Swarm Yaw: {self.swarm_yaw:.1f} deg")    
        except Exception as e:
            import traceback
            print(f"Failed to load trajectory file: {e}")
            traceback.print_exc()
        
        
    def cb_log(self):
        print("Logging command received!")
        self.logger.toggle_logging() # Start or stop logging
        # Update the GUI visuals based on the new state
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
                dpg.add_theme_color(dpg.mvThemeCol_Button, (100, 100, 100)) # Gray
                
        with dpg.theme(tag="theme_recording_active"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40)) # Bright Red
        
        with dpg.window(label="QUASAR Telemetry", width=1200, height=800):
            
            # ==========================================
            # 1. TOP DASHBOARD (Control Panel)
            # ==========================================
            with dpg.group(): 
                dpg.add_text("Toggle Variable Groups:")
                for chart_name, groups in self.plot_config.items():
                    with dpg.group(horizontal=True):
                        dpg.add_text(f"{chart_name}:", color=(200, 200, 200))
                        for group_name, variables in groups.items():
                            tags = [f"series_{var}" for var in variables]
                            dpg.add_checkbox(
                                label=group_name, default_value=True, 
                                callback=self._toggle_visibility, user_data=tags 
                            )
            
            dpg.add_spacer(height=10)
            
            # ==========================================
            # 2. STATUS PANEL
            # ==========================================
            with dpg.group(horizontal=True):
                dpg.add_text("STATUS |", color=(200, 200, 200))
                dpg.add_text("VBAT: -- V", tag="status_vbat")
                dpg.add_text("|")
                dpg.add_text("ARMED: FALSE", tag="status_armed", color=(150, 150, 150))
                dpg.add_text("FLYING: FALSE", tag="status_flying", color=(150, 150, 150))
                dpg.add_text("CRASHED: FALSE", tag="status_crashed", color=(150, 150, 150))
                dpg.add_text("LOCKED: FALSE", tag="status_locked", color=(150, 150, 150))
                dpg.add_button(
                            label="START RECORDING", 
                                width=150, 
                                height=40, 
                                tag="btn_record", 
                                callback=self.cb_log
                            )
                dpg.bind_item_theme("btn_record", "theme_recording_idle")
                
            dpg.add_spacer(height=10) 
            dpg.add_separator()
            
            
            # ==========================================
            #  COMMAND & CONTROL PANEL
            # ==========================================
            with dpg.group(horizontal=True):
                # Immediate Actions, land, takeoff, shutdown
                with dpg.group():
                    dpg.add_text("Flight Controls", color=(200, 200, 200))
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="TAKEOFF", width=100, height=40, callback=self.cb_takeoff)
                        dpg.add_button(label="LAND", width=100, height=40, callback=self.cb_land)
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
            # BOTTOM SECTION: PLOTS & TRACKBALL
            # ==========================================
            with dpg.group(horizontal=True):
                
                # --- LEFT SIDE: SUBPLOTS ---
                # width=-180 leaves exactly 180 pixels of space on the right
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
                                        line_label = f"{group_name} {var.upper()}"
                                        dpg.add_line_series(
                                            [], [], 
                                            label=line_label, tag=f"series_{var}", parent=y_axis
                                        )

                # --- RIGHT SIDE: TRACKBALL ---
                # This drops perfectly into the 180-pixel gap we left on the right
                with dpg.group():
                    dpg.add_text("Orientation", color=(200, 200, 200))
                    
                    with dpg.drawlist(width=150, height=150):
                        dpg.draw_rectangle([0,0], [150,150], color=[50,50,50], fill=[30,30,30])
                        
                        with dpg.draw_node(tag="trackball_node"):
                            dpg.draw_line([0,0], [0,0], color=[255, 50, 50], thickness=3, tag="tb_x")
                            dpg.draw_line([0,0], [0,0], color=[50, 255, 50], thickness=3, tag="tb_y")
                            dpg.draw_line([0,0], [0,0], color=[50, 150, 255], thickness=3, tag="tb_z")
                            
                    dpg.apply_transform("trackball_node", dpg.create_translation_matrix([75, 75]))

        # Populate the trajectory dropdown on startup
        self.cb_reload_trajectories() 
        
        dpg.create_viewport(title='QUASAR Testbed', width=1250, height=850)

        dpg.setup_dearpygui()
        dpg.show_viewport()
        
    def run(self):
        start_time = time.time()
        view_window_seconds = 5.0 
        
        last_t = 0.0 
        
        while dpg.is_dearpygui_running():
            snap = self.state.get_snapshot()
            #print(11343434343)
            
            # Only append if the data's timestamp is newer than the last one
            if snap['t'] > last_t:
                #print(14134)
                last_t = snap['t']
                current_time = time.time() - start_time
                self.time_data.append(current_time)
                
                x_list = list(self.time_data)
                
                # Update ALL lines in a single, flat loop!
                for var in self.tracked_vars:
                    self.history[var].append(snap[var])
                    dpg.set_value(f"series_{var}", [x_list, list(self.history[var])])
                
                #Update Trackball Orientation
                endpoints = self.get_trackball_lines(
                    [snap['eqx'], snap['eqy'], snap['eqz'], snap['eqw']]
                )
                # Update the p2 (endpoint) of each line. p1 stays at [0,0] (the center)
                dpg.configure_item("tb_x", p2=endpoints[0])
                dpg.configure_item("tb_y", p2=endpoints[1])
                dpg.configure_item("tb_z", p2=endpoints[2])
                
                #  UPDATE BATTERY DISPLAY
                vbat = snap['vbat']
                dpg.set_value("status_vbat", f"VBAT: {vbat:.2f} V")
                
                # Color code the battery (Assuming 1S LiPo: 4.2V max, 3.3V dead)
                if vbat < 3.3:
                    dpg.configure_item("status_vbat", color=(255, 50, 50)) # Red
                elif vbat < 3.6:
                    dpg.configure_item("status_vbat", color=(255, 200, 50)) # Yellow
                else:
                    dpg.configure_item("status_vbat", color=(50, 255, 50)) # Green

                #  UPDATE STATE FLAGS
                # ARMED FLAG
                if snap['armed']:
                    dpg.set_value("status_armed", "ARMED: TRUE")
                    dpg.configure_item("status_armed", color=(255, 50, 50)) # Danger Red
                else:
                    dpg.set_value("status_armed", "ARMED: FALSE")
                    dpg.configure_item("status_armed", color=(150, 150, 150)) # Safe Gray

                # FLYING FLAG
                if snap['flying']:
                    dpg.set_value("status_flying", "FLYING: TRUE")
                    dpg.configure_item("status_flying", color=(50, 255, 50)) # Green
                else:
                    dpg.set_value("status_flying", "FLYING: FALSE")
                    dpg.configure_item("status_flying", color=(150, 150, 150))

                # CRASHED FLAG
                if snap['crashed']:
                    dpg.set_value("status_crashed", "CRASHED: TRUE!")
                    dpg.configure_item("status_crashed", color=(255, 0, 0)) # Bright Red
                else:
                    dpg.set_value("status_crashed", "CRASHED: FALSE")
                    dpg.configure_item("status_crashed", color=(150, 150, 150))
                    
                if snap['locked']:
                    dpg.set_value("status_locked", "LOCKED: TRUE")
                    dpg.configure_item("status_locked", color=(255, 0, 0)) # Bright Red
                else:
                    dpg.set_value("status_locked", "LOCKED: FALSE")
                    dpg.configure_item("status_locked", color=(150, 150, 150))
            #print(13431413513535)
            # --- SCROLLING LOGIC (Runs every frame for smooth visuals) ---
            if self.time_data:
                latest_t = self.time_data[-1]
                min_t = latest_t - view_window_seconds
                
                if min_t < 0:
                    min_t = 0.0
                    
                dpg.set_axis_limits(self.master_x_axis, min_t, latest_t)
                
            
            dpg.render_dearpygui_frame()
            #print(13413413)
            
        dpg.destroy_context()
        


def test_gui():
    # 1. Initialize Shared State
    shared_state = SystemState()
    pose_queue = queue.Queue(maxsize=1)
    
    def mock_data():
        while True:
            with shared_state.lock:
                t = time.time()
                # Generates smooth waves for X, Y, and Z
                shared_state.estimate_pose.x, shared_state.estimate_pose.y, shared_state.estimate_pose.z = math.sin(t), math.cos(t), math.sin(t * 0.5)
                shared_state.time = t
            time.sleep(0.01) # ~50Hz
    # The 1-liner to spawn it
    threading.Thread(target=mock_data, daemon=True).start()
    mock_motive = start_mock_stream(pose_queue, shared_state)
    
    # 3. Start GUI on the Main Thread
    gui = QuasarGUI(shared_state)
    gui.setup_gui()
    gui.run() # This blocks and runs the while loop until you close the window

def start_gui(shared_state, command_queue=None, crazyflie=None):
    # Start the GUI on the main thread
    gui = QuasarGUI(shared_state, command_queue=command_queue)
    gui.setup_gui()
    gui.run() # This blocks and runs the while loop until you close the window
if __name__ == '__main__':
    # 1. Initialize Shared State
    shared_state = SystemState()
    
    # 2. Start Background Logging (Assuming 'cf' is your connected Crazyflie)
    # logger = TelemetryLogger(cf, shared_state)
    # logger.start_logging()
    
    # 3. Start GUI on the Main Thread
    gui = QuasarGUI(shared_state)
    gui.setup_gui()
    gui.run() # This blocks and runs the while loop until you close the window