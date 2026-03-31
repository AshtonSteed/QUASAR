import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import time
import sys

# 1. THE WINDOW FIX: Force Matplotlib to use the Tkinter backend BEFORE importing pyplot
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# 2. Try to import your trajectory math, catch error if file is missing
try:
    from trajectory_gen import generate_trajectory
except ImportError:
    # We use a basic Tk window just to show the error if the file is missing
    error_root = tk.Tk()
    error_root.withdraw()
    messagebox.showerror("Missing File", "Could not find 'trajectory_gen.py'.\nMake sure it is saved in the exact same folder as this script.")
    sys.exit()

def calculate_and_plot():
    try:
        status_label.config(text="Status: Calculating...", fg="blue")
        root.update()
        
        # Parse inputs
        shape_str = shape_var.get()
        shape_dict = {"Helix": 1, "Figure-8": 2, "Square": 3}
        shape_id = shape_dict[shape_str]
        
        width = float(width_entry.get())
        height = float(height_entry.get())
        speed = float(speed_entry.get())
        t_total = float(time_entry.get())
        
        # Generate 500 points
        t_steps = np.linspace(0, t_total, 500)
        p_data = np.zeros((len(t_steps), 3))
        
        for i, t in enumerate(t_steps):
            p, v, a, yaw = generate_trajectory(t, shape_id, width, height, speed, t_total)
            p_data[i, :] = p
            
        # Clear old plot
        ax.clear()
        
        # Plot the main path line (faded so the drone stands out)
        ax.plot(p_data[:,0], p_data[:,1], p_data[:,2], 'b-', linewidth=2, alpha=0.3, label=f"{shape_str} Path")
        
        # Plot Start (Green) and End (Black) markers
        ax.plot([p_data[0,0]], [p_data[0,1]], [p_data[0,2]], 'go', markersize=6, label="Start")
        ax.plot([p_data[-1,0]], [p_data[-1,1]], [p_data[-1,2]], 'ko', markersize=6, label="End")

        # Formatting
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Desired Trajectory: {shape_str}')

        # Force equal scaling
        max_range = np.array([p_data[:,0].max()-p_data[:,0].min(), 
                              p_data[:,1].max()-p_data[:,1].min(), 
                              p_data[:,2].max()-p_data[:,2].min()]).max() / 2.0
        mid_x = (p_data[:,0].max()+p_data[:,0].min()) * 0.5
        mid_y = (p_data[:,1].max()+p_data[:,1].min()) * 0.5
        mid_z = (p_data[:,2].max()+p_data[:,2].min()) * 0.5
        
        if max_range == 0: max_range = 1.0

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Create the empty Drone marker (Red Dot)
        drone_marker, = ax.plot([], [], [], 'ro', markersize=8, label="UAV")
        ax.legend()
        canvas.draw()

        # --- THE ANIMATION LOOP ---
        status_label.config(text="Status: Animating Flight...", fg="orange")
        root.update()
        
        # We don't want to draw all 500 frames (too slow), so we skip frames
        num_animation_frames = 60
        step = max(1, len(t_steps) // num_animation_frames)
        
        for i in range(0, len(t_steps), step):
            # Update the red dot's position
            drone_marker.set_data([p_data[i, 0]], [p_data[i, 1]])
            drone_marker.set_3d_properties([p_data[i, 2]])
            canvas.draw()
            root.update()
            time.sleep(0.01) # Small pause to make the animation visible
            
        # Snap the drone to the final position at the end
        drone_marker.set_data([p_data[-1, 0]], [p_data[-1, 1]])
        drone_marker.set_3d_properties([p_data[-1, 2]])
        canvas.draw()

        status_label.config(text="Status: Trajectory Generated & Animated!", fg="green")
        
    except Exception as e:
        status_label.config(text="Status: Error occurred.", fg="red")
        messagebox.showerror("Math Error", str(e))

def exit_app():
    root.quit()
    root.destroy()

# --- UI Setup ---
try:
    root = tk.Tk()
    root.title("QUASAR Trajectory Visualizer")
    root.geometry("900x600") 

    control_frame = tk.Frame(root, padx=20, pady=20)
    control_frame.pack(side=tk.LEFT, fill=tk.Y)

    plot_frame = tk.Frame(root)
    plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    # Controls
    tk.Label(control_frame, text="Trajectory Shape:").grid(row=0, column=0, sticky="w", pady=5)
    shape_var = tk.StringVar(value="Helix")
    shape_dropdown = ttk.Combobox(control_frame, textvariable=shape_var, values=["Helix", "Figure-8", "Square"], state="readonly")
    shape_dropdown.grid(row=0, column=1, pady=5)

    tk.Label(control_frame, text="Width/Radius (m):").grid(row=1, column=0, sticky="w", pady=5)
    width_entry = tk.Entry(control_frame)
    width_entry.insert(0, "2.0")
    width_entry.grid(row=1, column=1, pady=5)

    tk.Label(control_frame, text="Hover/Climb Height (m):").grid(row=2, column=0, sticky="w", pady=5)
    height_entry = tk.Entry(control_frame)
    height_entry.insert(0, "1.5")
    height_entry.grid(row=2, column=1, pady=5)

    tk.Label(control_frame, text="Speed Modifier:").grid(row=3, column=0, sticky="w", pady=5)
    speed_entry = tk.Entry(control_frame)
    speed_entry.insert(0, "1.0")
    speed_entry.grid(row=3, column=1, pady=5)

    tk.Label(control_frame, text="Total Sim Time (s):").grid(row=4, column=0, sticky="w", pady=5)
    time_entry = tk.Entry(control_frame)
    time_entry.insert(0, "20.0")
    time_entry.grid(row=4, column=1, pady=5)

    plot_btn = tk.Button(control_frame, text="Calculate & Animate Path", bg="lightblue", font=("Arial", 10, "bold"), command=calculate_and_plot)
    plot_btn.grid(row=5, column=0, columnspan=2, pady=15, ipadx=10, ipady=5)

    status_label = tk.Label(control_frame, text="Status: Ready", fg="black")
    status_label.grid(row=6, column=0, columnspan=2, pady=5)

    exit_btn = tk.Button(control_frame, text="Exit Application", bg="salmon", font=("Arial", 10, "bold"), command=exit_app)
    exit_btn.grid(row=7, column=0, columnspan=2, pady=30, ipadx=10, ipady=5)

    # Blank Plot Area
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Standby: Enter Parameters and Plot")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.draw()
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    root.mainloop()

except Exception as main_e:
    # If the window fails to build, print the exact error to the console
    print(f"CRITICAL UI ERROR: {main_e}")