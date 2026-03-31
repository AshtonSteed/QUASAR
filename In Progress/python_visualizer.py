import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from trajectory_gen import generate_trajectory

class TrajectoryUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Python UAV Trajectory Visualizer")

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

        self.btn = tk.Button(root, text="Plot Desired Path", command=self.plot_path, bg="blue", fg="white")
        self.btn.pack(pady=20)

    def plot_path(self):
        # Get UI Variables
        s_id = int(self.shape_id.get())
        w = float(self.width.get())
        h = float(self.height.get())
        spd = float(self.speed.get())
        t_tot = float(self.total_time.get())

        # Generate Time Array
        t_steps = np.linspace(0, t_tot, 500)
        p_data = np.zeros((len(t_steps), 3))

        # Calculate Trajectory
        for i, t in enumerate(t_steps):
            p, v, a, yaw = generate_trajectory(t, s_id, w, h, spd, t_tot)
            p_data[i, :] = p

        # --- Matplotlib 3D Plotting ---
        fig = plt.figure("Trajectory Preview")
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the path line
        ax.plot(p_data[:,0], p_data[:,1], p_data[:,2], 'r-', linewidth=2, label="Desired Path")
        
        # Plot Start (Green) and End (Black)
        ax.plot([p_data[0,0]], [p_data[0,1]], [p_data[0,2]], 'go', markersize=8, label="Start")
        ax.plot([p_data[-1,0]], [p_data[-1,1]], [p_data[-1,2]], 'ko', markersize=8, label="End")

        # Labels and formatting
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Trajectory Shape: {s_id}')
        ax.legend()

        # Force equal scaling so shapes don't distort
        max_range = np.array([p_data[:,0].max()-p_data[:,0].min(), 
                              p_data[:,1].max()-p_data[:,1].min(), 
                              p_data[:,2].max()-p_data[:,2].min()]).max() / 2.0
        mid_x = (p_data[:,0].max()+p_data[:,0].min()) * 0.5
        mid_y = (p_data[:,1].max()+p_data[:,1].min()) * 0.5
        mid_z = (p_data[:,2].max()+p_data[:,2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.show()

if __name__ == "__main__":
    root = tk.Tk()
    app = TrajectoryUI(root)
    root.mainloop()