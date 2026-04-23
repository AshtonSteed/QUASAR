import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def calculate_and_plot_trajectory_rmse(file_path):
    # Load the flight log
    df = pd.read_csv(file_path)

    # Verify all required columns exist
    required_columns = ['ex', 'ey', 'ez', 'sx', 'sy', 'sz', 'agent_id']
    if not all(col in df.columns for col in required_columns):
        raise ValueError(f"Missing one or more required columns: {required_columns}")

    # Dictionary to store results
    rmse_results = {}

    # Initialize a 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Group by agent_id to ensure we only calculate RMSE for consistent trajectories
    for agent, group in df.groupby('agent_id'):
        
        # Calculate the squared differences for x, y, and z axes
        sq_diff_x = (group['mx'] - group['sx'])**2
        sq_diff_y = (group['my'] - group['sy'])**2
        sq_diff_z = (group['mz'] - group['sz'])**2
        
        # Sum the squared differences, then find the mean across all points (MSE)
        mse = (sq_diff_x + sq_diff_y + sq_diff_z).mean()
        
        # Take the square root to get RMSE
        rmse = np.sqrt(mse)
        rmse_results[agent] = rmse
        
        # --- Plotting section ---
        # 1. Plot measured trajectory as a line
        ax.plot(group['mx'], group['my'], group['mz'], 
                label=f'Agent {agent} Measured (RMSE: {rmse:.4f}m)', 
                alpha=0.6, linewidth=2)
        
        ax.plot(group['sx'], group['sy'], group['sz'], 
                label=f'Agent {agent} Setpoint (RMSE: {rmse:.4f}m)', 
                alpha=0.6, linewidth=2)
        
        # 2. Plot the constant setpoint as a distinct marker
        # Since the setpoint is constant, we just grab the very first value (.iloc[0])
        sx_const = group['sx'].iloc[0]
        sy_const = group['sy'].iloc[0]
        sz_const = group['sz'].iloc[0]
        
        # Use scatter to place a large, distinct marker
        ax.scatter(sx_const, sy_const, sz_const, 
                   color='red', marker='X', s=150, 
                   label=f'Agent {agent} Setpoint', zorder=5)

    # Format the plot
    ax.set_title("3D Trajectory Static Hover", fontsize=22)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")
    ax.legend(loc='best')
    
    # Display the plot
    plt.tight_layout()
    plt.show()

    return rmse_results

# Example usage:
if __name__ == "__main__":
    file_path = '../../sittingtest.csv'
    results = calculate_and_plot_trajectory_rmse(file_path)

    print("3D Trajectory RMSE Results:")
    for agent, rmse in results.items():
        print(f"Agent {agent}: {rmse:.6f} meters")