import os
import pandas as pd
import numpy as np

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_metrics(csv_path):
    df = pd.read_csv(csv_path)
    
    # Calculate errors
    ex = df['x_d'] - df['x']
    ey = df['y_d'] - df['y']
    ez = df['z_d'] - df['z']
    epsi = wrap_angle(df['psi_d'] - df['psi'])
    
    # Euclidean position error
    e_pos = np.sqrt(ex**2 + ey**2 + ez**2)
    
    # Time delta
    t = df['t'].values
    dt = np.diff(t, prepend=t[0])
    
    # Metrics for position error
    mse_pos = np.mean(e_pos**2)
    rmse_pos = np.sqrt(mse_pos)
    mae_pos = np.mean(e_pos)
    itae_pos = np.sum(t * e_pos * dt)
    
    # Metrics for heading error
    mse_psi = np.mean(epsi**2)
    rmse_psi = np.sqrt(mse_psi)
    mae_psi = np.mean(np.abs(epsi))
    itae_psi = np.sum(t * np.abs(epsi) * dt)
    
    return {
        'Pos_MSE': mse_pos,
        'Pos_RMSE': rmse_pos,
        'Pos_MAE': mae_pos,
        'Pos_ITAE': itae_pos,
        'Psi_MSE': mse_psi,
        'Psi_RMSE': rmse_psi,
        'Psi_MAE': mae_psi,
        'Psi_ITAE': itae_psi
    }

def main():
    results_dir = '/home/bayisa/bluerov2_control_project/results'
    files = [f for f in os.listdir(results_dir) if f.endswith('.csv')]
    files.sort()
    
    all_results = []
    
    for filename in files:
        if filename == 'metrics_summary.csv':
            continue
            
        path = os.path.join(results_dir, filename)
        metrics = calculate_metrics(path)
        
        # Parse controller and trajectory names
        # Pattern: {controller}_{trajectory}_gazebo.csv
        
        # Trajectories are usually 'circle', 'figure8', 'spline'
        trajectories = ['circle', 'figure8', 'spline']
        traj = 'unknown'
        controller = filename.replace('_gazebo.csv', '')
        
        for t_name in trajectories:
            if t_name in filename:
                traj = t_name
                controller = filename.replace(f'_{t_name}_gazebo.csv', '')
                break
        
        all_results.append({
            'Controller': controller,
            'Trajectory': traj,
            **metrics
        })
    
    results_df = pd.DataFrame(all_results)
    
    # Save to CSV in results folder
    summary_path = os.path.join(results_dir, 'metrics_summary.csv')
    results_df.to_csv(summary_path, index=False)
    
    # Print as Markdown Table
    # Drop some columns for the printout to be readable in terminal
    print_cols = ['Controller', 'Trajectory', 'Pos_RMSE', 'Pos_MAE', 'Pos_ITAE', 'Psi_RMSE']
    print(results_df[print_cols].to_markdown(index=False))
    print(f"\nFull summary saved to: {summary_path}")

if __name__ == "__main__":
    main()
