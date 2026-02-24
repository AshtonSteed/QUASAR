import tkinter as tk
from tkinter import ttk, messagebox
import matlab.engine
import threading

def run_simulation():
    # Run in a separate thread so the UI doesn't freeze
    threading.Thread(target=execute_matlab_commands, daemon=True).start()

def execute_matlab_commands():
    try:
        status_label.config(text="Status: Connecting to MATLAB...", fg="blue")
        
        # 1. Parse inputs from the UI
        shape_str = shape_var.get()
        shape_dict = {"Helix": 1.0, "Figure-8": 2.0, "Square": 3.0}
        shape_id = shape_dict[shape_str]
        
        width = float(width_entry.get())
        height = float(height_entry.get())
        speed = float(speed_entry.get())
        t_total = float(time_entry.get())
        
        # 2. Connect to the shared MATLAB session
        eng = matlab.engine.connect_matlab()
        
        # 3. Push variables to the MATLAB Base Workspace
        eng.workspace['ui_shape_id'] = shape_id
        eng.workspace['ui_width'] = width
        eng.workspace['ui_height'] = height
        eng.workspace['ui_speed'] = speed
        eng.workspace['ui_time'] = t_total
        
        # 4. Configure and run Simulink
        model_name = 'crazyflie_sim' # Ensure this matches your .slx file name exactly
        status_label.config(text=f"Status: Simulating {model_name}...", fg="orange")
        
        # Load system and set stop time
        eng.eval(f"if bdIsLoaded('{model_name}') == 0; load_system('{model_name}'); end", nargout=0)
        eng.eval(f"set_param('{model_name}', 'StopTime', '{t_total}')", nargout=0)
        
        # Run the model
        eng.eval(f"sim('{model_name}')", nargout=0)
        
        status_label.config(text="Status: Flight Complete!", fg="green")
        messagebox.showinfo("Success", "Simulink flight complete.")
        
    except Exception as e:
        status_label.config(text="Status: Error occurred.", fg="red")
        messagebox.showerror("MATLAB Engine Error", str(e))

# --- UI Setup ---
root = tk.Tk()
root.title("UAV Ground Control Station")
root.geometry("350x350")
root.configure(padx=20, pady=20)

# Trajectory Shape
tk.Label(root, text="Trajectory Shape:").grid(row=0, column=0, sticky="w", pady=5)
shape_var = tk.StringVar(value="Helix")
shape_dropdown = ttk.Combobox(root, textvariable=shape_var, values=["Helix", "Figure-8", "Square"], state="readonly")
shape_dropdown.grid(row=0, column=1, pady=5)

# Width / Radius
tk.Label(root, text="Width/Radius (m):").grid(row=1, column=0, sticky="w", pady=5)
width_entry = tk.Entry(root)
width_entry.insert(0, "2.0")
width_entry.grid(row=1, column=1, pady=5)

# Hover / Climb Height
tk.Label(root, text="Hover/Climb Height (m):").grid(row=2, column=0, sticky="w", pady=5)
height_entry = tk.Entry(root)
height_entry.insert(0, "1.5")
height_entry.grid(row=2, column=1, pady=5)

# Speed / Frequency
tk.Label(root, text="Speed Modifier:").grid(row=3, column=0, sticky="w", pady=5)
speed_entry = tk.Entry(root)
speed_entry.insert(0, "1.0")
speed_entry.grid(row=3, column=1, pady=5)

# Total Sim Time
tk.Label(root, text="Total Sim Time (s):").grid(row=4, column=0, sticky="w", pady=5)
time_entry = tk.Entry(root)
time_entry.insert(0, "20.0")
time_entry.grid(row=4, column=1, pady=5)

# Fly Button
fly_btn = tk.Button(root, text="Upload Trajectory & Fly", bg="lightblue", font=("Arial", 10, "bold"), command=run_simulation)
fly_btn.grid(row=5, column=0, columnspan=2, pady=20, ipadx=10, ipady=5)

# Status Label
status_label = tk.Label(root, text="Status: Ready", fg="black")
status_label.grid(row=6, column=0, columnspan=2)

root.mainloop()