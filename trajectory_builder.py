import os
import sys
import json
import math
import dearpygui.dearpygui as dpg

from GSCore.tools.trajectory_math import TrajectoryGenerator

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
        """Scrapes the active UI values and saves them to the internal state to prevent data loss on refresh."""
        for seg in self.segments:
            if dpg.does_item_exist(f"seg_{seg['id']}_n"):
                seg["n"] = dpg.get_value(f"seg_{seg['id']}_n")
                seg["speed"] = dpg.get_value(f"seg_{seg['id']}_speed")
                
                if seg["type"] == "Linear":
                    seg["x"] = dpg.get_value(f"seg_{seg['id']}_x")
                    seg["y"] = dpg.get_value(f"seg_{seg['id']}_y")
                    seg["z"] = dpg.get_value(f"seg_{seg['id']}_z")
                
                elif seg["type"] == "Helical":
                    seg["cx"] = dpg.get_value(f"seg_{seg['id']}_cx")
                    seg["cy"] = dpg.get_value(f"seg_{seg['id']}_cy")
                    seg["tz"] = dpg.get_value(f"seg_{seg['id']}_tz")
                    seg["r"] = dpg.get_value(f"seg_{seg['id']}_r")
                    seg["sweep"] = dpg.get_value(f"seg_{seg['id']}_sweep")
                    
                elif seg["type"] == "Circular":
                    seg["plane"] = dpg.get_value(f"seg_{seg['id']}_plane")
                    seg["cx"] = dpg.get_value(f"seg_{seg['id']}_cx")
                    seg["cy"] = dpg.get_value(f"seg_{seg['id']}_cy")
                    seg["cz"] = dpg.get_value(f"seg_{seg['id']}_cz")
                    seg["r"] = dpg.get_value(f"seg_{seg['id']}_r")

    def add_linear_segment(self):
        self.sync_ui_to_state()
        self.segment_counter += 1
        self.segments.append({
            "id": self.segment_counter,
            "type": "Linear",
            "x": 0.0, "y": 0.0, "z": 1.0, 
            "speed": 1.0, "n": 50
        })
        self.refresh_segment_ui()

    def add_helical_segment(self):
        self.sync_ui_to_state()
        self.segment_counter += 1
        self.segments.append({
            "id": self.segment_counter,
            "type": "Helical",
            "cx": 1.0, "cy": 0.0, "tz": 1.0, 
            "r": 1.0, "sweep": 6.28, 
            "speed": 1.0, "n": 100
        })
        self.refresh_segment_ui()

    def add_circular_segment(self):
        self.sync_ui_to_state()
        self.segment_counter += 1
        self.segments.append({
            "id": self.segment_counter,
            "type": "Circular",
            "plane": "XY",
            "cx": 0.0, "cy": 0.0, "cz": 1.0, 
            "r": 2.0, "speed": 1.0, "n": 100
        })
        self.refresh_segment_ui()
        
    def remove_segment(self, sender, app_data, user_data):
        self.sync_ui_to_state()
        seg_id = user_data
        self.segments = [s for s in self.segments if s["id"] != seg_id]
        self.refresh_segment_ui()

    def refresh_segment_ui(self):
        """Rebuilds the UI list of segments based on the internal state."""
        dpg.delete_item("segment_container", children_only=True)
        
        for i, seg in enumerate(self.segments):
            with dpg.collapsing_header(label=f"Segment {i+1}: {seg['type']}", parent="segment_container", default_open=True):
                
                dpg.add_input_float(label="Speed (m/s)", default_value=seg["speed"], tag=f"seg_{seg['id']}_speed", width=100)
                
                if seg["type"] == "Linear":
                    dpg.add_input_float(label="Target X", default_value=seg["x"], tag=f"seg_{seg['id']}_x", width=100)
                    dpg.add_input_float(label="Target Y", default_value=seg["y"], tag=f"seg_{seg['id']}_y", width=100)
                    dpg.add_input_float(label="Target Z", default_value=seg["z"], tag=f"seg_{seg['id']}_z", width=100)
                    dpg.add_input_int(label="N Points", default_value=seg["n"], tag=f"seg_{seg['id']}_n", width=100)
                
                elif seg["type"] == "Helical":
                    dpg.add_input_float(label="Center X", default_value=seg["cx"], tag=f"seg_{seg['id']}_cx", width=100)
                    dpg.add_input_float(label="Center Y", default_value=seg["cy"], tag=f"seg_{seg['id']}_cy", width=100)
                    dpg.add_input_float(label="Target Z", default_value=seg["tz"], tag=f"seg_{seg['id']}_tz", width=100)
                    dpg.add_input_float(label="Radius", default_value=seg["r"], tag=f"seg_{seg['id']}_r", width=100)
                    dpg.add_input_float(label="Sweep (rad)", default_value=seg["sweep"], tag=f"seg_{seg['id']}_sweep", width=100)
                    dpg.add_input_int(label="N Points", default_value=seg["n"], tag=f"seg_{seg['id']}_n", width=100)
                    
                elif seg["type"] == "Circular":
                    dpg.add_combo(label="Plane", items=["XY", "XZ", "YZ"], default_value=seg["plane"], tag=f"seg_{seg['id']}_plane", width=100)
                    dpg.add_input_float(label="Center X", default_value=seg["cx"], tag=f"seg_{seg['id']}_cx", width=100)
                    dpg.add_input_float(label="Center Y", default_value=seg["cy"], tag=f"seg_{seg['id']}_cy", width=100)
                    dpg.add_input_float(label="Center Z", default_value=seg["cz"], tag=f"seg_{seg['id']}_cz", width=100)
                    dpg.add_input_float(label="Radius", default_value=seg["r"], tag=f"seg_{seg['id']}_r", width=100)
                    dpg.add_input_int(label="N Points", default_value=seg["n"], tag=f"seg_{seg['id']}_n", width=100)
                
                dpg.add_button(label="Remove Segment", user_data=seg["id"], callback=self.remove_segment)
                dpg.add_separator()
        
        self.recalculate_trajectory()

    def recalculate_trajectory(self):
        """Reads the UI, chains the segments, adds timing data, and updates plots."""
        self.sync_ui_to_state()
        
        current_pos = (
            dpg.get_value("start_x"), 
            dpg.get_value("start_y"), 
            dpg.get_value("start_z"), 
            0.0 # Time t=0
        )
        
        all_waypoints = []
        current_time = 0.0
        
        for seg in self.segments:
            try:
                # 1. Generate base geometry
                if seg["type"] == "Linear":
                    target = (seg["x"], seg["y"], seg["z"], 0.0)
                    raw_wp = TrajectoryGenerator.linear(current_pos, target, seg["n"])
                    
                elif seg["type"] == "Helical":
                    center = (seg["cx"], seg["cy"])
                    raw_wp = TrajectoryGenerator.helical(current_pos, center, seg["r"], seg["sweep"], seg["tz"], seg["n"])

                elif seg["type"] == "Circular":
                    raw_wp = []
                    cx, cy, cz = seg["cx"], seg["cy"], seg["cz"]
                    r = seg["r"]
                    plane = seg["plane"]
                    n_pts = max(2, seg["n"])
                    
                    # Generate a full 360-degree circle oriented on the chosen plane
                    for i in range(n_pts):
                        theta = (i / (n_pts - 1)) * 2 * math.pi
                        dx = r * math.cos(theta)
                        dy = r * math.sin(theta)
                        
                        if plane == "XY":
                            raw_wp.append((cx + dx, cy + dy, cz))
                        elif plane == "XZ":
                            raw_wp.append((cx + dx, cy, cz + dy))
                        elif plane == "YZ":
                            raw_wp.append((cx, cy + dx, cz + dy))

                # 2. Inject timestamps based on distance and speed
                speed = max(0.001, seg["speed"]) # Prevent divide by zero
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
                    timed_wp.append((p[0], p[1], p[2], current_time))

                all_waypoints.extend(timed_wp)
                current_pos = timed_wp[-1] 
                
            except Exception as e:
                print(f"Error calculating segment {seg['id']}: {e}")
                
        self.generated_waypoints = all_waypoints
        
        # Auto-update the duration box based on physics math
        if all_waypoints:
            dpg.set_value("input_duration", round(all_waypoints[-1][3], 2))
            
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
        save_dir = os.path.abspath(os.path.join(current_dir, '..', 'trajectories'))
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
        
        self.update_plots()
        
        dpg.start_dearpygui()
        dpg.destroy_context()

if __name__ == "__main__":
    builder = TrajectoryBuilderGUI()
    builder.run()