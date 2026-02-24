# Whatever driver script is needed to display GUI 
# Probably have seperate files for, plotting telemetry, sending commands, etc.

import queue

import dearpygui.dearpygui as dpg
from collections import deque
import math, time, threading

from GSCore.drivers.mock_motive_client import start_mock_stream
from common_classes import SystemState

class QuasarGUI:
    def __init__(self, shared_state, window_width=1000):
        self.state = shared_state
        
        # Rolling buffers for the plot (X-axis and Y-axes)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        
        # We will start with just Position data for simplicity, 
        # but you can easily add 'p', 'q', 'r', 'vbat', etc.
        self.history = {
            'ex': deque(maxlen=self.max_points),
            'ey': deque(maxlen=self.max_points),
            'ez': deque(maxlen=self.max_points),
            'mx': deque(maxlen=self.max_points),
            'my': deque(maxlen=self.max_points),
            'mz': deque(maxlen=self.max_points),
        }
        
    def _toggle_visibility(self, sender, app_data, user_data):
        """Hides or shows multiple line series based on the checkbox state"""
        is_checked = app_data
        series_tags = user_data # This is now a list of strings
        
        # Loop through every tag in the list and apply the show/hide state
        for tag in series_tags:
            # Optional failsafe: check if the item exists before configuring it
            if dpg.does_item_exist(tag): 
                dpg.configure_item(tag, show=is_checked)
    
    def setup_gui(self):
        dpg.create_context()
        
        with dpg.window(label="UAV Telemetry", width=800, height=600):
            
            # --- CONTROL PANEL ---
            dpg.add_text("Toggle Variables:")
            with dpg.group(horizontal=True):
               # Group 1: Position
                dpg.add_checkbox(
                    label="Position (X,Y,Z)", 
                    default_value=True, 
                    callback=self._toggle_visibility, 
                    user_data=["series_ex", "series_ey", "series_ez"] # Passed as a list!
                )
                dpg.add_checkbox(label="Motive (X Y Z)", default_value=True, 
                                 callback=self._toggle_visibility, user_data=["series_mx", "series_my", "series_mz"])
            
            dpg.add_spacer(height=10)
            
            # --- LIVE PLOT ---
            with dpg.plot(label="UAV State Estimate", height=-1, width=-1):
                # Add Legend
                dpg.add_plot_legend()

                # Add X Axis
                self.x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
                
                # Add Y Axis
                # Add Y Axis
                y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Position (m)")
                
                # Explicitly set the parent to the y_axis ID
                dpg.add_line_series([], [], label="X (m)", tag="series_ex", parent=y_axis)
                dpg.add_line_series([], [], label="Y (m)", tag="series_ey", parent=y_axis)
                dpg.add_line_series([], [], label="Z (m)", tag="series_ez", parent=y_axis)
                dpg.add_line_series([], [], label="mX (m)", tag="series_mx", parent=y_axis)
                dpg.add_line_series([], [], label="mY (m)", tag="series_my", parent=y_axis)
                dpg.add_line_series([], [], label="mZ (m)", tag="series_mz", parent=y_axis)
                

        dpg.create_viewport(title='UAV Testbed', width=850, height=650)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        
    def run(self):
        start_time = time.time()
        view_window_seconds = 5.0 
        
        last_plotted_time = 0.0 
        
        while dpg.is_dearpygui_running():
            snap = self.state.get_snapshot()
            
            # Only append if the data's timestamp is newer than the last one
            if snap['t'] > last_plotted_time:
                last_plotted_time = snap['t']
                
                # Calculate 'real' computer time
                current_time = time.time() - start_time
                
                # Append to buffers
                self.time_data.append(current_time)
                self.history['ex'].append(snap['ex'])
                self.history['ey'].append(snap['ey'])
                self.history['ez'].append(snap['ez'])
                self.history['mx'].append(snap['mx'])
                self.history['my'].append(snap['my'])
                self.history['mz'].append(snap['mz'])
                
                # Update the DPG line series
                x_list = list(self.time_data)
                dpg.set_value("series_ex", [x_list, list(self.history['ex'])])
                dpg.set_value("series_ey", [x_list, list(self.history['ey'])])
                dpg.set_value("series_ez", [x_list, list(self.history['ez'])])
                dpg.set_value("series_mx", [x_list, list(self.history['mx'])])
                dpg.set_value("series_my", [x_list, list(self.history['my'])])
                dpg.set_value("series_mz", [x_list, list(self.history['mz'])])
            
            # --- SCROLLING LOGIC (Runs every frame for smooth visuals) ---
            if self.time_data:
                latest_t = self.time_data[-1]
                min_t = latest_t - view_window_seconds
                
                if min_t < 0:
                    min_t = 0.0
                    
                dpg.set_axis_limits(self.x_axis, min_t, latest_t)
                
            # Render the frame (this will run at max FPS, but data only updates at 50Hz)
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