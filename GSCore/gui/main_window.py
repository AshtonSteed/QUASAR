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

    def cb_emergency_stop(self):
        # 1. Kill any active trajectory stream in GUI
        self.cancel_trajectory = True
        
        # 2. Send hardware halt command
        if self.cmd_queue:
            self.cmd_queue.emergency_stop()
            print("EMERGENCY STOP: Flight Aborted and Motors Deactivated.")

    def _stream_waypoints(self, waypoints, segment_duration):
        """This runs in the background. It won't freeze your buttons!"""
        for i, wp in enumerate(waypoints):
            if self.cancel_trajectory:
                print("Aborting stream.")
                break
                
            x, y, z, yaw = wp
            # The 'linear=True' is key for the Crazyflie high-level commander
            self.cmd_queue.goto(x, y, z, yaw=yaw, duration=segment_duration, linear=True)
            time.sleep(segment_duration)

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
            # FULL 3D TRANSLATION & SWARM ROTATION
            # ==========================================
            
            # 1. Get the physical start point from Mocap
            with self.state.lock:
                actual_start_x = self.state.estimate_pose.x
                actual_start_y = self.state.estimate_pose.y
                actual_start_z = self.state.estimate_pose.z 
            
            # 2. Get the trajectory's designed start position
            designed_start_x = raw_waypoints[0][0]
            designed_start_y = raw_waypoints[0][1]
            designed_start_z = raw_waypoints[0][2]      
            
            # Calculate the Z altitude offset
            offset_z = actual_start_z - designed_start_z 
            
            # 3. Get the Swarm Rotation offset and convert to radians
            swarm_yaw_deg = dpg.get_value("input_swarm_yaw")
            theta = math.radians(swarm_yaw_deg)
            
            translated_waypoints = []
            
            for wp in raw_waypoints:
                # A. Shift the designed path so its start point is at (0,0)
                dx = wp[0] - designed_start_x
                dy = wp[1] - designed_start_y
                
                # B. Apply 2D Rotation Matrix to the X/Y coordinates
                rot_x = (dx * math.cos(theta)) - (dy * math.sin(theta))
                rot_y = (dx * math.sin(theta)) + (dy * math.cos(theta))
                
                # C. Translate the rotated point to the drone's physical Mocap location
                new_x = actual_start_x + rot_x
                new_y = actual_start_y + rot_y
                
                # D. Apply the Z offset so the shape floats at the drone's current altitude
                new_z = wp[2] + offset_z # <-- NEW: Shift the playbook up/down
                
                # E. Apply Yaw to the drone's nose
                designed_yaw = wp[3] if len(wp) > 3 else 0.0
                new_yaw = designed_yaw + theta
                
                # Format for CommandQueue: (X, Y, Z, Yaw)
                translated_waypoints.append((new_x, new_y, new_z, new_yaw))
                
            # ==========================================
            
            # Send the safely translated/rotated points to the 50Hz state machine
            self.cmd_queue.execute_trajectory(translated_waypoints, duration)
            print(f"Executing playbook: {data.get('name', file_name)} (Swarm Offset: {swarm_yaw_deg} deg)")
            print(f"Applied Offsets -> X: {actual_start_x-designed_start_x:+.2f}m, Y: {actual_start_y-designed_start_y:+.2f}m, Z: {offset_z:+.2f}m")
            
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
                        dpg.add_button(label="EMERGENCY STOP", width=150, height=40, callback=self.cb_emergency_stop)
                        
                        # Bind a theme specifically to the Kill Switch to make it bright red
                        with dpg.theme() as kill_theme:
                            with dpg.theme_component(dpg.mvButton):
                                dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 50, 50))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 30, 30))
                        dpg.bind_item_theme(dpg.last_item(), kill_theme)
                    

                dpg.add_spacer(width=30)

                # High-Level Trajectory Playbook Loader
                with dpg.group():
                    dpg.add_text("Trajectory Playbook", color=(200, 200, 200))
                    
                    # Row 1: File Selection & Swarm Offset
                    with dpg.group(horizontal=True):
                        dpg.add_text("Select Path:")
                        dpg.add_combo(items=[], tag="input_traj_file", width=150)
                        dpg.add_button(label="RELOAD", callback=self.cb_reload_trajectories)
                        
                        dpg.add_text("  Swarm Yaw Offset (deg):")
                        dpg.add_input_float(tag="input_swarm_yaw", default_value=0.0, width=80, step=15.0)

                    # Row 2: The Execute Button
                    dpg.add_spacer(height=5)
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="EXECUTE TRAJECTORY", tag="btn_execute", width=250, height=30, callback=self.cb_goto)
                        
                        # Apply visual theme to the Execute button safely using its tag
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