import dearpygui.dearpygui as dpg
from collections import deque
import math, time, threading
import queue
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
        
    def get_trackball_lines(self, quat, scale=50):
        rot = Rotation.from_quat(quat)
        rot_matrix = rot.as_matrix()
        axes = np.array([
            [1, 0, 0], # X axis (Red)
            [0, 1, 0], # Y axis (Green)
            [0, 0, 1]  # Z axis (Blue)
        ])
        rotated_axes = axes @ rot_matrix.T
        lines_2d = []
        for axis in rotated_axes:
            screen_x = axis[0] * scale
            screen_y = -axis[2] * scale 
            lines_2d.append([screen_x, screen_y])
        return lines_2d 

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
            agent.command_queue.takeoff(height=1.0, duration=2.0)
            
    def cb_land(self):
        for agent in self._get_target_agents():
            agent.command_queue.land(height=0.0, duration=2.0)

    def cb_emergency_stop(self):
        # ALWAYS kill everything in the swarm, regardless of dropdown selection
        self.cancel_trajectory = True
        for agent in self.swarm_dict.values():
            agent.command_queue.emergency_stop()
        print("EMERGENCY STOP: Swarm Flight Aborted and Motors Deactivated.")

    def _stream_waypoints(self, agent, waypoints, segment_duration):
        for i, wp in enumerate(waypoints):
            if self.cancel_trajectory:
                break
            x, y, z, yaw = wp
            agent.command_queue.goto(x, y, z, yaw=yaw, duration=segment_duration, linear=True)
            time.sleep(segment_duration)

    def cb_goto(self):
        tgt_x = dpg.get_value("input_goto_x")
        tgt_y = dpg.get_value("input_goto_y")
        tgt_z = dpg.get_value("input_goto_z")
        tgt_duration = dpg.get_value("input_goto_time")
        traj_type = dpg.get_value("input_traj_type")
        n_waypoints = max(2, dpg.get_value("input_n_waypoints"))
        
        self.cancel_trajectory = True
        for t in self.trajectory_threads:
            if t.is_alive(): t.join(timeout=0.1)
        self.cancel_trajectory = False
        self.trajectory_threads.clear()

        segment_duration = tgt_duration / n_waypoints
        
        # Calculate routes and spawn threads for all targeted agents
        for agent in self._get_target_agents():
            with agent.state.lock:
                start_pos = (agent.state.estimate_pose.x, agent.state.estimate_pose.y, agent.state.estimate_pose.z, 0.0)
                
            if traj_type == "Linear":
                # Warning: If SWARM is selected, they will all converge to the exact same (tgt_x, tgt_y)
                # In the future, apply offsets (e.g. tgt_x + offset_x) for swarm maneuvers.
                target_pos = (tgt_x, tgt_y, tgt_z, 0.0)
                waypoints = TrajectoryGenerator.linear(start_pos, target_pos, n_waypoints)
            elif traj_type == "Helical":
                radius = dpg.get_value("input_radius")
                sweep_angle = dpg.get_value("input_sweep")
                center = (tgt_x, tgt_y)
                waypoints = TrajectoryGenerator.helical(start_pos, center, radius, sweep_angle, tgt_z, n_waypoints)
            
            t = threading.Thread(target=self._stream_waypoints, args=(agent, waypoints, segment_duration), daemon=True)
            t.start()
            self.trajectory_threads.append(t)

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
                        dpg.add_button(label="EMERGENCY STOP", width=150, height=40, callback=self.cb_emergency_stop)
                        with dpg.theme() as kill_theme:
                            with dpg.theme_component(dpg.mvButton):
                                dpg.add_theme_color(dpg.mvThemeCol_Button, (200, 40, 40))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 50, 50))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 30, 30))
                        dpg.bind_item_theme(dpg.last_item(), kill_theme)

                dpg.add_spacer(width=30)

                with dpg.group():
                    dpg.add_text("Trajectory Command Setup", color=(200, 200, 200))
                    with dpg.group(horizontal=True):
                        dpg.add_text("X:"); dpg.add_input_float(tag="input_goto_x", default_value=0.0, width=90, step=0.1)
                        dpg.add_text("Y:"); dpg.add_input_float(tag="input_goto_y", default_value=0.0, width=90, step=0.1)
                        dpg.add_text("Z:"); dpg.add_input_float(tag="input_goto_z", default_value=1.0, width=90, step=0.1)
                    
                    with dpg.group(horizontal=True):
                        dpg.add_text("Type:"); dpg.add_combo(items=["Linear", "Helical"], default_value="Linear", tag="input_traj_type", width=100)
                        dpg.add_text("Time(s):"); dpg.add_input_float(tag="input_goto_time", default_value=3.0, width=90, step=0.5)
                        dpg.add_text("N-Pts:"); dpg.add_input_int(tag="input_n_waypoints", default_value=15, width=90, step=1)
                        
                    with dpg.group(horizontal=True):
                        dpg.add_text("Radius (m):"); dpg.add_input_float(tag="input_radius", default_value=1.0, width=90, step=0.1)
                        dpg.add_text("Sweep (rad):"); dpg.add_input_float(tag="input_sweep", default_value=3.14, width=90, step=0.1)

                    dpg.add_spacer(height=5)
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="EXECUTE TRAJECTORY", width=250, height=30, callback=self.cb_goto)
                        with dpg.theme() as exec_theme:
                            with dpg.theme_component(dpg.mvButton):
                                dpg.add_theme_color(dpg.mvThemeCol_Button, (40, 150, 40)) 
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (50, 200, 50))
                                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (30, 120, 30))
                        dpg.bind_item_theme(dpg.last_item(), exec_theme)
                        
            dpg.add_separator()
            dpg.add_spacer(height=10)
            
            # ==========================================
            # LAYOUT 1: SWARM OVERVIEW (Default)
            # ==========================================
            with dpg.group(tag="group_swarm_view", show=True):
                dpg.add_text("Swarm Spatial Overview", color=(150, 150, 255))
                with dpg.plot(label="Top-Down Map", width=-1, height=450):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="X Position (m)", tag="swarm_x_axis")
                    dpg.set_axis_limits("swarm_x_axis", -3.0, 3.0) # Typical Motive Volume limits
                    
                    y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Y Position (m)", tag="swarm_y_axis")
                    dpg.set_axis_limits("swarm_y_axis", -3.0, 3.0)
                    
                    # Create one scatter series for each drone
                    for a_id in self.swarm_dict.keys():
                        dpg.add_scatter_series([], [], label=a_id, tag=f"scatter_{a_id}", parent=y_axis)


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

                    # 2A. Update scatter plot if we are in Swarm View
                    if current_view == "SWARM":
                        dpg.set_value(f"scatter_{a_id}", [[snap['ex']], [snap['ey']]])

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