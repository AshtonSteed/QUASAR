import os
import sys
import json
import math
import dearpygui.dearpygui as dpg

from GSCCore.core.trajectory_math import TrajectoryGenerator

class TrajectoryBuilderGUI:
    def __init__(self):
        self.segments = []
        self.segment_counter = 0
        self.generated_waypoints = []
        
        # --- 3D View Camera State ---
        self.view_yaw = 0.785     # 45 degrees
        self.view_pitch = 0.523   # 30 degrees
        self.view_scale = 25.0    # Zoom level
        self.is_dragging = False
        self.start_yaw = self.view_yaw
        self.start_pitch = self.view_pitch
        
    def sync_ui_to_state(self):
        """Scrapes the active UI values and saves them to the internal state using ABSOLUTE coordinates."""
        for seg in self.segments:
            sid = seg['id']
            if dpg.does_item_exist(f"seg_{sid}_n"):
                seg["n"] = dpg.get_value(f"seg_{sid}_n")
                seg["speed"] = dpg.get_value(f"seg_{sid}_speed")
                
                # We use absolute coordinates (x, y, z, cx, cy)
                if seg["type"] == "Linear":
                    seg["x"] = dpg.get_value(f"seg_{sid}_x")
                    seg["y"] = dpg.get_value(f"seg_{sid}_y")
                    seg["z"] = dpg.get_value(f"seg_{sid}_z")
                
                elif seg["type"] == "Helical":
                    seg["cx"] = dpg.get_value(f"seg_{sid}_cx")
                    seg["cy"] = dpg.get_value(f"seg_{sid}_cy")
                    seg["tz"] = dpg.get_value(f"seg_{sid}_tz")
                    seg["r"] = dpg.get_value(f"seg_{sid}_r")
                    seg["sweep_deg"] = dpg.get_value(f"seg_{sid}_sweep") # Stored in degrees
                    
                elif seg["type"] == "Circular":
                    seg["plane"] = dpg.get_value(f"seg_{sid}_plane")
                    seg["cx"] = dpg.get_value(f"seg_{sid}_cx")
                    seg["cy"] = dpg.get_value(f"seg_{sid}_cy")
                    seg["cz"] = dpg.get_value(f"seg_{sid}_cz")
                    seg["r"] = dpg.get_value(f"seg_{sid}_r")

    def _get_end_state(self, index_before):
        """Calculates the absolute X,Y,Z,Yaw state at the end of a specific segment index."""
        start_yaw = dpg.get_value("start_yaw") if dpg.does_item_exist("start_yaw") else 0.0
        current_state = (
            dpg.get_value("start_x") if dpg.does_item_exist("start_x") else 0.0,
            dpg.get_value("start_y") if dpg.does_item_exist("start_y") else 0.0,
            dpg.get_value("start_z") if dpg.does_item_exist("start_z") else 0.0,
            start_yaw
        )
        
        if index_before < 0 or not self.segments:
            return current_state
            
        # Fast-forward calculation to find the exact end point of the target segment
        for i in range(min(index_before + 1, len(self.segments))):
            seg = self.segments[i]
            if seg["type"] == "Linear":
                target = (seg.get("x",0), seg.get("y",0), seg.get("z",0), current_state[3])
                raw = TrajectoryGenerator.linear(current_state, target, 2)
            elif seg["type"] == "Helical":
                center = (seg.get("cx",0), seg.get("cy",0))
                sweep_rad = math.radians(seg.get("sweep_deg", 360.0))
                raw = TrajectoryGenerator.helical(current_state, center, seg.get("r",1), sweep_rad, seg.get("tz",0), 2)
            elif seg["type"] == "Circular":
                raw = [current_state, current_state] # Circular ends where it starts
                
            current_state = raw[-1]
            
        return current_state

    # --- ADD, REMOVE, MOVE, INSERT LOGIC ---
    def get_seg_index(self, seg_id):
        for i, s in enumerate(self.segments):
            if s["id"] == seg_id: return i
        return -1

    def move_segment(self, sender, app_data, user_data):
        seg_id, direction = user_data
        self.sync_ui_to_state()
        idx = self.get_seg_index(seg_id)
        new_idx = idx + direction
        if 0 <= new_idx < len(self.segments):
            self.segments[idx], self.segments[new_idx] = self.segments[new_idx], self.segments[idx]
            self.refresh_segment_ui()

    def insert_segment(self, sender, app_data, user_data):
        seg_id, seg_type = user_data
        self.sync_ui_to_state()
        idx = self.get_seg_index(seg_id)
        
        start_state = self._get_end_state(idx - 1)
        sx, sy, sz, _ = start_state
        
        self.segment_counter += 1
        new_seg = {"id": self.segment_counter, "type": seg_type, "speed": 1.0, "n": 50}
        
        if seg_type == "Linear":
            new_seg.update({"x": sx + 1.0, "y": sy, "z": sz})
        elif seg_type == "Helical":
            new_seg.update({"cx": sx + 1.0, "cy": sy, "tz": sz, "r": 1.0, "sweep_deg": 360.0})
        elif seg_type == "Circular":
            new_seg.update({"plane": "XY", "cx": sx, "cy": sy, "cz": sz, "r": 2.0})
            
        self.segments.insert(idx, new_seg)
        self.refresh_segment_ui()

    def add_linear_segment(self):
        self.sync_ui_to_state()
        start_state = self._get_end_state(len(self.segments) - 1)
        sx, sy, sz, _ = start_state
        
        self.segment_counter += 1
        self.segments.append({
            "id": self.segment_counter, "type": "Linear",
            "x": sx, "y": sy, "z": sz + 1.0, "speed": 1.0, "n": 50
        })
        self.refresh_segment_ui()

    def add_helical_segment(self):
        self.sync_ui_to_state()
        start_state = self._get_end_state(len(self.segments) - 1)
        sx, sy, sz, _ = start_state
        
        self.segment_counter += 1
        self.segments.append({
            "id": self.segment_counter, "type": "Helical",
            "cx": sx + 1.0, "cy": sy, "tz": sz, "r": 1.0, "sweep_deg": 360.0, "speed": 1.0, "n": 100
        })
        self.refresh_segment_ui()

    def add_circular_segment(self):
        self.sync_ui_to_state()
        start_state = self._get_end_state(len(self.segments) - 1)
        sx, sy, sz, _ = start_state
        
        self.segment_counter += 1
        self.segments.append({
            "id": self.segment_counter, "type": "Circular",
            "plane": "XY", "cx": sx, "cy": sy, "cz": sz, "r": 2.0, "speed": 1.0, "n": 100
        })
        self.refresh_segment_ui()
        
    def remove_segment(self, sender, app_data, user_data):
        self.sync_ui_to_state()
        seg_id = user_data
        self.segments = [s for s in self.segments if s["id"] != seg_id]
        self.refresh_segment_ui()

    # --- UI & CALCULATIONS ---
    def refresh_segment_ui(self):
        """Rebuilds the UI list of segments based on the internal state."""
        dpg.delete_item("segment_container", children_only=True)
        
        for i, seg in enumerate(self.segments):
            sid = seg['id']
            with dpg.collapsing_header(label=f"[{i+1}] {seg['type']}", parent="segment_container", default_open=True):
                
                with dpg.group(horizontal=True):
                    dpg.add_button(label="^", user_data=(sid, -1), callback=self.move_segment)
                    dpg.add_button(label="v", user_data=(sid, 1), callback=self.move_segment)
                    dpg.add_button(label="+LIN", user_data=(sid, "Linear"), callback=self.insert_segment)
                    dpg.add_button(label="+HEL", user_data=(sid, "Helical"), callback=self.insert_segment)
                    dpg.add_button(label="+CIR", user_data=(sid, "Circular"), callback=self.insert_segment)
                    dpg.add_button(label="Remove", user_data=sid, callback=self.remove_segment)
                
                dpg.add_spacer(height=5)
                
                with dpg.group(horizontal=True):
                    dpg.add_input_float(label="Speed (m/s)", default_value=seg["speed"], tag=f"seg_{sid}_speed", width=100)
                    dpg.add_input_int(label="N Points", default_value=seg["n"], tag=f"seg_{sid}_n", width=100)
                
                # UI uses Absolute Labels
                if seg["type"] == "Linear":
                    dpg.add_input_float(label="Target X", default_value=seg["x"], tag=f"seg_{sid}_x", width=100)
                    dpg.add_input_float(label="Target Y", default_value=seg["y"], tag=f"seg_{sid}_y", width=100)
                    dpg.add_input_float(label="Target Z", default_value=seg["z"], tag=f"seg_{sid}_z", width=100)
                
                elif seg["type"] == "Helical":
                    dpg.add_input_float(label="Abs Center X", default_value=seg["cx"], tag=f"seg_{sid}_cx", width=100)
                    dpg.add_input_float(label="Abs Center Y", default_value=seg["cy"], tag=f"seg_{sid}_cy", width=100)
                    dpg.add_input_float(label="Target Z", default_value=seg["tz"], tag=f"seg_{sid}_tz", width=100)
                    dpg.add_input_float(label="Radius", default_value=seg["r"], tag=f"seg_{sid}_r", width=100)
                    dpg.add_input_float(label="Sweep (deg)", default_value=seg["sweep_deg"], tag=f"seg_{sid}_sweep", width=100)
                    
                elif seg["type"] == "Circular":
                    dpg.add_combo(label="Plane", items=["XY", "XZ", "YZ"], default_value=seg["plane"], tag=f"seg_{sid}_plane", width=100)
                    dpg.add_input_float(label="Abs CX", default_value=seg["cx"], tag=f"seg_{sid}_cx", width=100)
                    dpg.add_input_float(label="Abs CY", default_value=seg["cy"], tag=f"seg_{sid}_cy", width=100)
                    dpg.add_input_float(label="Abs CZ", default_value=seg["cz"], tag=f"seg_{sid}_cz", width=100)
                    dpg.add_input_float(label="Radius", default_value=seg["r"], tag=f"seg_{sid}_r", width=100)
                
                dpg.add_separator()
        
        self.recalculate_trajectory()

    def recalculate_trajectory(self):
        """Reads the UI, chains the segments using ABSOLUTE logic, and updates plots."""
        self.sync_ui_to_state()
        
        start_yaw = dpg.get_value("start_yaw") if dpg.does_item_exist("start_yaw") else 0.0

        current_pos = (
            dpg.get_value("start_x"), 
            dpg.get_value("start_y"), 
            dpg.get_value("start_z"), 
            start_yaw, 
            0.0 # Time
        )
        
        all_waypoints = []
        current_time = 0.0
        
        for seg in self.segments:
            try:
                # Strip time for the math generator: (X, Y, Z, Yaw)
                math_start_pos = current_pos[:4] 
                
                # 1. Generate geometry using ABSOLUTE Targets/Centers
                if seg["type"] == "Linear":
                    target = (seg["x"], seg["y"], seg["z"], math_start_pos[3])
                    raw_wp = TrajectoryGenerator.linear(math_start_pos, target, seg["n"])
                    
                elif seg["type"] == "Helical":
                    center = (seg["cx"], seg["cy"])
                    sweep_rad = math.radians(seg["sweep_deg"]) # Convert UI degrees to Math radians
                    raw_wp = TrajectoryGenerator.helical(math_start_pos, center, seg["r"], sweep_rad, seg["tz"], seg["n"])

                elif seg["type"] == "Circular":
                    raw_wp = []
                    cx = seg["cx"]
                    cy = seg["cy"]
                    cz = seg["cz"]
                    r = seg["r"]
                    plane = seg["plane"]
                    n_pts = max(2, seg["n"])
                    
                    for i in range(n_pts):
                        theta = (i / (n_pts - 1)) * 2 * math.pi
                        dx = r * math.cos(theta)
                        dy = r * math.sin(theta)
                        
                        if plane == "XY":
                            raw_wp.append((cx + dx, cy + dy, cz, 0.0))
                        elif plane == "XZ":
                            raw_wp.append((cx + dx, cy, cz + dy, 0.0))
                        elif plane == "YZ":
                            raw_wp.append((cx, cy + dx, cz + dy, 0.0))

                # 2. Inject timestamps based on distance and speed
                speed = max(0.001, seg.get("speed", 1.0))
                timed_wp = []
                
                for i, p in enumerate(raw_wp):
                    if i == 0:
                        prev_p = all_waypoints[-1] if all_waypoints else current_pos
                    else:
                        prev_p = raw_wp[i-1]
                        
                    dist = math.sqrt(
                        (p[0] - prev_p[0])**2 + 
                        (p[1] - prev_p[1])**2 + 
                        (p[2] - prev_p[2])**2
                    )
                    
                    current_time += (dist / speed)
                    timed_wp.append((p[0], p[1], p[2], p[3], current_time))

                all_waypoints.extend(timed_wp)
                current_pos = timed_wp[-1] # The end of this becomes the start of the next
                
            except Exception as e:
                import traceback
                print(f"Error calculating segment {seg['id']}: {e}")
                traceback.print_exc()
                
        self.generated_waypoints = all_waypoints
        
        if all_waypoints:
            if dpg.does_item_exist("input_duration"):
                dpg.set_value("input_duration", round(all_waypoints[-1][4], 2))
            
        self.update_plots()

    # --- 3D Engine Methods ---
    def project_3d_to_2d(self, x, y, z):
        cx, cy = 375, 375  

        x_r = x * math.cos(self.view_yaw) - y * math.sin(self.view_yaw)
        y_r = x * math.sin(self.view_yaw) + y * math.cos(self.view_yaw)

        y_p = y_r * math.cos(self.view_pitch) - z * math.sin(self.view_pitch)
        z_p = y_r * math.sin(self.view_pitch) + z * math.cos(self.view_pitch)

        screen_x = cx + x_r * self.view_scale
        screen_y = cy - z_p * self.view_scale  

        return [screen_x, screen_y]

    def update_plots(self):
        if dpg.does_item_exist("lbl_total_pts"):
            dpg.set_value("lbl_total_pts", f"Total Waypoints: {len(self.generated_waypoints)}")
        
        if not dpg.does_item_exist("plot_3d_drawlist"):
            return
            
        dpg.delete_item("plot_3d_drawlist", children_only=True)

        grid_size = 10
        for i in range(-grid_size, grid_size + 1):
            p1 = self.project_3d_to_2d(-grid_size, i, 0)
            p2 = self.project_3d_to_2d(grid_size, i, 0)
            dpg.draw_line(p1, p2, color=(100, 100, 100, 100), thickness=1, parent="plot_3d_drawlist")
            p1 = self.project_3d_to_2d(i, -grid_size, 0)
            p2 = self.project_3d_to_2d(i, grid_size, 0)
            dpg.draw_line(p1, p2, color=(100, 100, 100, 100), thickness=1, parent="plot_3d_drawlist")

        origin = self.project_3d_to_2d(0, 0, 0)
        dpg.draw_line(origin, self.project_3d_to_2d(grid_size, 0, 0), color=(255, 70, 70, 255), thickness=2, parent="plot_3d_drawlist") 
        dpg.draw_line(origin, self.project_3d_to_2d(0, grid_size, 0), color=(70, 255, 70, 255), thickness=2, parent="plot_3d_drawlist")  
        dpg.draw_line(origin, self.project_3d_to_2d(0, 0, grid_size), color=(70, 70, 255, 255), thickness=2, parent="plot_3d_drawlist")  
        
        dpg.draw_text(self.project_3d_to_2d(grid_size+1, 0, 0), "X", color=(255,70,70,255), size=18, parent="plot_3d_drawlist")
        dpg.draw_text(self.project_3d_to_2d(0, grid_size+1, 0), "Y", color=(70,255,70,255), size=18, parent="plot_3d_drawlist")
        dpg.draw_text(self.project_3d_to_2d(0, 0, grid_size+1), "Z", color=(70,70,255,255), size=18, parent="plot_3d_drawlist")

        if not self.generated_waypoints:
            return

        pts = [self.project_3d_to_2d(wp[0], wp[1], wp[2]) for wp in self.generated_waypoints]
        dpg.draw_polyline(pts, color=(0, 255, 255, 255), thickness=2, parent="plot_3d_drawlist")
        
        if pts:
            dpg.draw_circle(pts[0], radius=5, color=(0, 255, 0, 255), fill=(0, 255, 0, 255), parent="plot_3d_drawlist")
            dpg.draw_circle(pts[-1], radius=5, color=(255, 0, 0, 255), fill=(255, 0, 0, 255), parent="plot_3d_drawlist")

    # --- Mouse Control Handlers ---
    def on_mouse_click(self, sender, app_data):
        if dpg.is_item_hovered("plot_3d_drawlist"):
            self.is_dragging = True
            self.start_yaw = self.view_yaw
            self.start_pitch = self.view_pitch

    def on_mouse_release(self, sender, app_data):
        self.is_dragging = False

    def on_mouse_drag(self, sender, app_data):
        if self.is_dragging:
            dx = app_data[1]
            dy = app_data[2]
            self.view_yaw = self.start_yaw + dx * 0.01
            self.view_pitch = self.start_pitch - dy * 0.01
            self.update_plots()

    def on_mouse_wheel(self, sender, app_data):
        if dpg.is_item_hovered("plot_3d_drawlist"):
            self.view_scale += app_data * 2.0
            self.view_scale = max(2.0, self.view_scale)
            self.update_plots()

    def scan_trajectories(self):
        """Scans the trajectories folder and populates the load dropdown."""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        save_dir = os.path.abspath(os.path.join(current_dir, 'trajectories'))
        os.makedirs(save_dir, exist_ok=True)
        
        # Find all json files
        files = [f for f in os.listdir(save_dir) if f.endswith('.json')]
        
        if dpg.does_item_exist("combo_load_file"):
            dpg.configure_item("combo_load_file", items=files)
            if files: 
                dpg.set_value("combo_load_file", files[0])

    def load_trajectory(self):
        """Loads the selected JSON file and rebuilds the UI state."""
        filename = dpg.get_value("combo_load_file")
        if not filename: return
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(current_dir, 'trajectories', filename)
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                
            # 1. Restore the segment definitions (the math instructions)
            self.segments = data.get("segment_definitions", [])
            
            # 2. Fast-forward the segment ID counter so new additions don't collide
            if self.segments:
                self.segment_counter = max(seg["id"] for seg in self.segments)
            else:
                self.segment_counter = 0
                
            # 3. Update the export text boxes at the bottom
            if dpg.does_item_exist("input_filename"):
                dpg.set_value("input_filename", filename)
            if dpg.does_item_exist("input_traj_name"):
                dpg.set_value("input_traj_name", data.get("name", "Loaded Path"))
                
            # 4. Trigger the UI to rebuild itself and recalculate the 3D plot
            self.refresh_segment_ui()
            
            if dpg.does_item_exist("lbl_save_status"):
                dpg.set_value("lbl_save_status", f"Loaded {filename}!")
                
        except Exception as e:
            print(f"Failed to load {filename}: {e}")

    def save_trajectory(self):
        self.sync_ui_to_state()
        
        if not self.generated_waypoints:
            print("No trajectory to save!")
            return
            
        filename = dpg.get_value("input_filename")
        if not filename.endswith(".json"):
            filename += ".json"
            
        name = dpg.get_value("input_traj_name")
        duration = dpg.get_value("input_duration")
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        save_dir = os.path.abspath(os.path.join(current_dir, 'trajectories'))
        os.makedirs(save_dir, exist_ok=True)
        filepath = os.path.join(save_dir, filename)

        traj_data = {
            "name": name,
            "total_duration": duration,
            "num_points": len(self.generated_waypoints),
            "segment_definitions": self.segments,
            "waypoints": self.generated_waypoints
        }

        with open(filepath, 'w') as f:
            json.dump(traj_data, f, indent=4)
        print(f"SAVED: '{name}' to {filepath}")
        if dpg.does_item_exist("lbl_save_status"):
            dpg.set_value("lbl_save_status", f"Saved successfully to {filename}!")

    def run(self):
        dpg.create_context()
        
        with dpg.handler_registry():
            dpg.add_mouse_click_handler(button=0, callback=self.on_mouse_click)
            dpg.add_mouse_release_handler(button=0, callback=self.on_mouse_release)
            dpg.add_mouse_drag_handler(button=0, callback=self.on_mouse_drag)
            dpg.add_mouse_wheel_handler(callback=self.on_mouse_wheel)
        
        with dpg.window(label="QUASAR Trajectory Builder", width=1200, height=800):
            with dpg.group(horizontal=True):
                
                with dpg.child_window(width=400):
                    
                    dpg.add_text("0. Load Existing Playbook", color=(200,200,200))
                    with dpg.group(horizontal=True):
                        dpg.add_combo(items=[], tag="combo_load_file", width=180)
                        dpg.add_button(label="SCAN", callback=self.scan_trajectories)
                        dpg.add_button(label="LOAD", callback=self.load_trajectory)
                    dpg.add_separator()
                    
                    dpg.add_text("1. Absolute Start Point", color=(200,200,200))
                    with dpg.group(horizontal=True):
                        dpg.add_input_float(label="X", default_value=0.0, tag="start_x", width=80)
                        dpg.add_input_float(label="Y", default_value=0.0, tag="start_y", width=80)
                        dpg.add_input_float(label="Z", default_value=0.0, tag="start_z", width=80)
                    dpg.add_separator()
                    
                    dpg.add_text("2. Build Path", color=(200,200,200))
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="+ Add Linear", callback=self.add_linear_segment)
                        dpg.add_button(label="+ Add Helical", callback=self.add_helical_segment)
                        dpg.add_button(label="+ Add Circular", callback=self.add_circular_segment)
                    
                    dpg.add_spacer(height=10)
                    
                    with dpg.child_window(height=400, tag="segment_container"):
                        pass
                    
                    dpg.add_spacer(height=10)
                    dpg.add_button(label="PREVIEW / RECALCULATE", width=-1, height=40, callback=self.recalculate_trajectory)
                    dpg.add_text("Total Waypoints: 0", tag="lbl_total_pts", color=(100, 255, 100))
                    
                    dpg.add_separator()
                    dpg.add_text("3. Export to Playbook", color=(200,200,200))
                    dpg.add_input_text(label="File Name", default_value="complex_path.json", tag="input_filename")
                    dpg.add_input_text(label="Display Name", default_value="Complex Path", tag="input_traj_name")
                    dpg.add_input_float(label="Calculated Duration (s)", default_value=0.0, tag="input_duration")
                    dpg.add_spacer(height=5)
                    dpg.add_button(label="SAVE TO JSON", width=-1, height=40, callback=self.save_trajectory)
                    dpg.add_text("", tag="lbl_save_status", color=(100, 255, 100))

                with dpg.child_window(width=-1):
                    with dpg.group(horizontal=True):
                        dpg.add_text("3D Trajectory View", color=(255, 255, 255))
                        dpg.add_text("(Drag to Rotate, Scroll to Zoom)", color=(150, 150, 150))
                    
                    dpg.add_spacer(height=5)
                    
                    with dpg.drawlist(width=750, height=750, tag="plot_3d_drawlist"):
                        pass

        dpg.create_viewport(title='Trajectory Builder', width=1250, height=850)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        
        self.scan_trajectories()  # Populate load dropdown on startup
        self.update_plots()
        
        dpg.start_dearpygui()
        dpg.destroy_context()

if __name__ == "__main__":
    builder = TrajectoryBuilderGUI()
    builder.run()