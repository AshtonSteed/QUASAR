import tkinter as tk
from tkinter import messagebox
import matlab.engine

class TrajectoryUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UAV Trajectory Visualizer")
        
        # Connect to MATLAB
        try:
            print("Connecting to shared MATLAB engine...")
            self.eng = matlab.engine.connect_matlab()
            print("Connected!")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Ensure 'matlab.engine.shareEngine' is run in MATLAB.\n{e}")
            self.root.destroy()

        # UI Layout
        tk.Label(root, text="Select Shape (1:Helix, 2:Fig8, 3:Square):").pack()
        self.shape_id = tk.Scale(root, from_=1, to=3, orient='horizontal')
        self.shape_id.pack()

        tk.Label(root, text="Width (m):").pack()
        self.width = tk.Scale(root, from_=0, to=10, resolution=0.1, orient='horizontal')
        self.width.set(2.0)
        self.width.pack()

        tk.Label(root, text="Height (m):").pack()
        self.height = tk.Scale(root, from_=0, to=10, resolution=0.1, orient='horizontal')
        self.height.set(1.5)
        self.height.pack()

        tk.Label(root, text="Speed Factor:").pack()
        self.speed = tk.Scale(root, from_=0.1, to=5.0, resolution=0.1, orient='horizontal')
        self.speed.set(1.0)
        self.speed.pack()

        tk.Label(root, text="Total Time (s):").pack()
        self.total_time = tk.Scale(root, from_=1, to=60, orient='horizontal')
        self.total_time.set(10)
        self.total_time.pack()

        self.btn = tk.Button(root, text="Plot Desired Path", command=self.plot_path, bg="green", fg="white")
        self.btn.pack(pady=20)

    def plot_path(self):
        try:
            # Inject variables into MATLAB workspace
            self.eng.workspace['ui_shape_id'] = float(self.shape_id.get())
            self.eng.workspace['ui_width'] = float(self.width.get())
            self.eng.workspace['ui_height'] = float(self.height.get())
            self.eng.workspace['ui_speed'] = float(self.speed.get())
            self.eng.workspace['ui_time'] = float(self.total_time.get())
            
            # Run the MATLAB plotting script
            print("Sending data to MATLAB...")
            self.eng.plot_desired(nargout=0)
        except Exception as e:
            messagebox.showerror("Error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = TrajectoryUI(root)
    root.mainloop()