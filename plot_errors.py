import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse


def process_and_plot(pose_file, command_file):
    # Load the data
    pose_df = pd.read_csv(pose_file)
    command_df = pd.read_csv(command_file)

    # Strip whitespace from column names
    pose_df.columns = pose_df.columns.str.strip()
    command_df.columns = command_df.columns.str.strip()

    # Compute relative time
    command_df['dt'] = command_df['ts'].diff().fillna(0)
    command_time = command_df['ts'] - command_df['ts'].iloc[0]

    # Integrate velocity commands for position (ideal path)
    integrated_x = [0]
    integrated_y = [0]
    integrated_theta = [0]
    for i in range(1, len(command_df)):
        theta_new = integrated_theta[-1] + command_df['w'].iloc[i] * command_df['dt'].iloc[i]
        integrated_theta.append(theta_new)
        
        dx = command_df['v'].iloc[i] * np.cos(theta_new) * command_df['dt'].iloc[i]
        dy = command_df['v'].iloc[i] * np.sin(theta_new) * command_df['dt'].iloc[i]
        
        integrated_x.append(integrated_x[-1] + dx)
        integrated_y.append(integrated_y[-1] + dy)

    fig, axs = plt.subplots(1, 5, figsize=(18, 5))

    # Plot 1: Position comparison (integrated commands, odom measurements, and Kalman Filter estimate)
    axs[0].plot(integrated_x, integrated_y, label='Integrated Commands', linestyle='dotted')
    if 'odom_x' in pose_df and 'odom_y' in pose_df:
        axs[0].plot(pose_df['odom_x'], pose_df['odom_y'], label='Pose', linestyle='dashed')
    axs[0].plot(pose_df['kf_x'], pose_df['kf_y'], label='Kalman Filter Estimate')
    axs[0].set_title('Position Comparison')
    axs[0].set_xlabel('X Position')
    axs[0].set_ylabel('Y Position')
    axs[0].legend()
    axs[0].grid(True)
    axs[0].set_aspect('equal', adjustable='box')

    # Plot 2: Acceleration (Ax) comparison
    axs[1].plot(pose_df['stamp'] - pose_df['stamp'].iloc[0], pose_df['imu_ax'], label='IMU Ax')
    axs[1].plot(pose_df['stamp'] - pose_df['stamp'].iloc[0], pose_df['kf_ax'], label='Kalman Filter Ax', linestyle='dotted')
    axs[1].set_title('Ax Comparison')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Ax')
    axs[1].legend()
    axs[1].grid(True)

    # Plot 3: Acceleration (Ay) comparison
    axs[2].plot(pose_df['stamp'] - pose_df['stamp'].iloc[0], pose_df['imu_ay'], label='IMU Ay')
    axs[2].plot(pose_df['stamp'] - pose_df['stamp'].iloc[0], pose_df['kf_ay'], label='Kalman Filter Ay', linestyle='dotted')
    axs[2].set_title('Ay Comparison')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Ay')
    axs[2].legend()
    axs[2].grid(True)

    # Plot 4: Velocity (Vx) comparison
    axs[3].plot(command_time, command_df['v'], label='Command Vx', linestyle='dotted')
    axs[3].plot(pose_df['stamp'] - pose_df['stamp'].iloc[0], pose_df['kf_vx'], label='Kalman Filter Vx')
    axs[3].set_title('Vx Comparison')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Vx')
    axs[3].legend()
    axs[3].grid(True)

    # Plot 5: Angular Velocity (W) comparison
    axs[4].plot(command_time, command_df['w'], label='Command W', linestyle='dotted')
    axs[4].plot(pose_df['stamp'] - pose_df['stamp'].iloc[0], pose_df['kf_w'], label='Kalman Filter W')
    axs[4].set_title('W Comparison')
    axs[4].set_xlabel('Time (s)')
    axs[4].set_ylabel('W')
    axs[4].legend()
    axs[4].grid(True)

    plt.suptitle(f'EKF Results: Point Q={0.5} R={0.8}', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.95])    
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot EKF results from data files.")
    parser.add_argument("pose_file", type=str, help="Path to the pose CSV file")
    parser.add_argument("command_file", type=str, help="Path to the command CSV file")
    args = parser.parse_args()

    process_and_plot(args.pose_file, args.command_file)
