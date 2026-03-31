# Whatever driver script is needed to display GUI 
# Probably have seperate files for, plotting telemetry, sending commands, etc.

import queue

import dearpygui.dearpygui as dpg
from collections import deque
import math, time, threading
import numpy as np
from scipy.spatial.transform import Rotation

from GSCore.core.commands import DroneCmd, DroneCommand
from GSCore.core.trajectory_math import TrajectoryGenerator
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

    def cb_goto(self):
        if not self.cmd_queue: return
        
        # 1. Read standard Target/Timing inputs
        tgt_x = dpg.get_value("input_goto_x")
        tgt_y = dpg.get_value("input_goto_y")
        tgt_z = dpg.get_value("input_goto_z")
        tgt_duration = dpg.get_value("input_goto_time")
            
        # 2. Read Trajectory-specific inputs
        traj_type = dpg.get_value("input_traj_type")
        n_waypoints = dpg.get_value("input_n_waypoints")
        
        # Ensure N is at least 2, creating a real path
        if n_waypoints < 2: 
            n_waypoints = 2
            
        # 3. Start Point (P0), current position from shared state 
        with self.state.lock:
            start_pos = (
                self.state.estimate_pose.x, 
                self.state.estimate_pose.y, 
                self.state.estimate_pose.z, 
                0.0 # Default starting yaw
            )
            
        # 4. Route to correct trajectory generator based on dropdown  
        if traj_type == "Linear":
            target_pos = (tgt_x, tgt_y, tgt_z, 0.0)
            waypoints = TrajectoryGenerator.linear(start_pos, target_pos, n_waypoints)
            
        elif traj_type == "Helical":
            radius = dpg.get_value("input_radius")
            sweep_angle = dpg.get_value("input_sweep")
            center = (tgt_x, tgt_y)
            
            waypoints = TrajectoryGenerator.helical(
                start_pos, center, radius, sweep_angle, tgt_z, n_waypoints
            )
            
        # 5. Calculate pacing and spawn the thread
        segment_duration = tgt_duration / n_waypoints
        
        self.cancel_trajectory = True
        if self.trajectory_thread and self.trajectory_thread.is_alive():
            self.trajectory_thread.join(timeout=0.1)
            
        self.cancel_trajectory = False
        self.trajectory_thread = threading.Thread(
            target=self._stream_waypoints,
            args=(waypoints, segment_duration),
            daemon=True
        )
        self.trajectory_thread.start()
        
        
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

                # High-Level Trajectory Command Inputs
                with dpg.group():
                    dpg.add_text("Trajectory Command Setup", color=(200, 200, 200))
                    
                    # Row 1: Target Coordinates
                    with dpg.group(horizontal=True):
                        dpg.add_text("X:")
                        dpg.add_input_float(tag="input_goto_x", default_value=0.0, width=90, step=0.1)
                        dpg.add_text("Y:")
                        dpg.add_input_float(tag="input_goto_y", default_value=0.0, width=90, step=0.1)
                        dpg.add_text("Z:")
                        dpg.add_input_float(tag="input_goto_z", default_value=1.0, width=90, step=0.1)
                    
                    # Row 2: General Flight Parameters
                    with dpg.group(horizontal=True):
                        dpg.add_text("Type:")
                        dpg.add_combo(items=["Linear", "Helical"], default_value="Linear", tag="input_traj_type", width=100)
                        dpg.add_text("Time(s):")
                        dpg.add_input_float(tag="input_goto_time", default_value=3.0, width=90, step=0.5)
                        dpg.add_text("N-Pts:")
                        dpg.add_input_int(tag="input_n_waypoints", default_value=15, width=90, step=1)
                        
                    # Row 3: Helical-Specific Parameters
                    with dpg.group(horizontal=True):
                        dpg.add_text("Radius (m):")
                        dpg.add_input_float(tag="input_radius", default_value=1.0, width=90, step=0.1)
                        dpg.add_text("Sweep (rad):")
                        dpg.add_input_float(tag="input_sweep", default_value=3.14, width=90, step=0.1)

                    # Row 4: The Execute Button
                    dpg.add_spacer(height=5)
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="EXECUTE TRAJECTORY", width=250, height=30, callback=self.cb_goto)
                        
                        # Add a visual theme to the Execute button
                        with dpg.theme() as exec_theme:
                            with dpg.theme_component(dpg.mvButton):
                                dpg.add_theme_color(dpg.mvThemeCol_Button, (40, 150, 40)) # Greenish
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (50, 200, 50))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (30, 120, 30))
                        dpg.bind_item_theme(dpg.last_item(), exec_theme)
                        
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

        dpg.create_viewport(title='QUASAR Testbed', width=1250, height=850)

        dpg.setup_dearpygui()
        dpg.show_viewport()
        
    def run(self):
        start_time = time.time()
        view_window_seconds = 5.0 
        
        last_t = 0.0 
        
        while dpg.is_dearpygui_running():
            snap = self.state.get_snapshot()
            
            # Only append if the data's timestamp is newer than the last one
            if snap['t'] > last_t:
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
        
            # --- SCROLLING LOGIC (Runs every frame for smooth visuals) ---
            if self.time_data:
                latest_t = self.time_data[-1]
                min_t = latest_t - view_window_seconds
                
                if min_t < 0:
                    min_t = 0.0
                    
                dpg.set_axis_limits(self.master_x_axis, min_t, latest_t)
                
            
            dpg.render_dearpygui_frame()
            
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