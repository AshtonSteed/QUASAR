import pandas as pd
import numpy as np

def calculate_trajectory_rmse(file_path):
    # Load the flight log
    df = pd.read_csv(file_path)

    # Verify all required columns exist
    required_columns = ['mx', 'my', 'mz', 'sx', 'sy', 'sz', 'agent_id']
    if not all(col in df.columns for col in required_columns):
        raise ValueError(f"Missing one or more required columns: {required_columns}")

    # Dictionary to store results
    rmse_results = {}

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
        
    return rmse_results

# Example usage:
file_path = '../../WobbleTest.csv'
results = calculate_trajectory_rmse(file_path)

print("3D Trajectory RMSE Results:")
for agent, rmse in results.items():
    print(f"Agent {agent}: {rmse:.6f} meters")