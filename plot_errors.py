import os
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from utilities import FileReader


def plot_errors(directory, controller_type="P"):
    angular_file = os.path.join(directory, "angular.csv")
    linear_file = os.path.join(directory, "linear.csv")
    robot_pose_file = os.path.join(directory, "robot_pose.csv")
    
    fig, axes = plt.subplots(3, 2, figsize=(18, 15))
    
    fig.suptitle(f"Controller Type: {controller_type}", fontsize=16)
    
    # --- Angular Data Plots ---
    headers, angular_values = FileReader(angular_file).read_file()
    print(f"Plotting data from {angular_file}")
    time_list = [(val[-1] - angular_values[0][-1]) * 1e-9 for val in angular_values]
    
    # Plot e-t and e_dot-t with dual y-axes
    ax1 = axes[0, 0]
    ax1.plot(time_list, [val[0] for val in angular_values], 'o-', label="e (Error)", color="blue")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("e (Error)", color="blue")
    ax1.tick_params(axis="y", labelcolor="blue")
    ax1.set_title("Angular: Error (e) and Error Derivative (e_dot) over Time")
    ax1.grid()
    
    ax2 = ax1.twinx()
    ax2.plot(time_list, [val[1] for val in angular_values], 's-', label="e_dot (Derivative of Error)", color="red")
    ax2.set_ylabel("e_dot (Derivative of Error)", color="red")
    ax2.tick_params(axis="y", labelcolor="red")
    
    # Plot e vs e_dot for angular
    axes[0, 1].plot([val[0] for val in angular_values], [val[1] for val in angular_values], 'd-', label="e vs e_dot")
    axes[0, 1].set_title("Angular: Error vs Error Derivative")
    axes[0, 1].set_xlabel("e (Error)")
    axes[0, 1].set_ylabel("e_dot (Derivative of Error)")
    axes[0, 1].legend()
    axes[0, 1].grid()

    # --- Linear Data Plots ---
    headers, linear_values = FileReader(linear_file).read_file()
    print(f"Plotting data from {linear_file}")
    time_list = [(val[-1] - linear_values[0][-1]) * 1e-9 for val in linear_values]
    
    # Plot e-t and e_dot-t with dual y-axes
    ax3 = axes[1, 0]
    ax3.plot(time_list, [val[0] for val in linear_values], 'o-', label="e (Error)", color="green")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("e (Error)", color="green")
    ax3.tick_params(axis="y", labelcolor="green")
    ax3.set_title("Linear: Error (e) and Error Derivative (e_dot) over Time")
    ax3.grid()
    
    ax4 = ax3.twinx()
    ax4.plot(time_list, [val[1] for val in linear_values], 's-', label="e_dot (Derivative of Error)", color="purple")
    ax4.set_ylabel("e_dot (Derivative of Error)", color="purple")
    ax4.tick_params(axis="y", labelcolor="purple")
    
    # Plot e vs e_dot for linear
    axes[1, 1].plot([val[0] for val in linear_values], [val[1] for val in linear_values], 'd-', label="e vs e_dot")
    axes[1, 1].set_title("Linear: Error vs Error Derivative")
    axes[1, 1].set_xlabel("e (Error)")
    axes[1, 1].set_ylabel("e_dot (Derivative of Error)")
    axes[1, 1].legend()
    axes[1, 1].grid()

    # --- Robot Pose Data Plots ---
    headers, robot_pose_values = FileReader(robot_pose_file).read_file()
    print(f"Plotting data from {robot_pose_file}")
    time_list = [(val[-1] - robot_pose_values[0][-1]) * 1e-9 for val in robot_pose_values]
    
    # Plot theta on the left y-axis and x, y on the right y-axis
    ax5 = axes[2, 0]
    ax5.plot(time_list, [val[2] for val in robot_pose_values], label="theta over Time", color="magenta")
    ax5.set_ylabel("theta (rad)", color="magenta")
    ax5.tick_params(axis="y", labelcolor="magenta")
    ax5.set_xlabel("Time (s)")
    ax5.set_title("Robot Pose: States x, y, theta over Time")
    ax5.grid()

    ax6 = ax5.twinx()
    ax6.plot(time_list, [val[0] for val in robot_pose_values], label="x over Time", color="cyan")
    ax6.plot(time_list, [val[1] for val in robot_pose_values], label="y over Time", color="orange")
    ax6.set_ylabel("x, y (meters)", color="cyan")
    ax6.tick_params(axis="y", labelcolor="cyan")

    # Add legend for dual y-axis plot
    ax5.legend(loc="upper left")
    ax6.legend(loc="upper right")
    
    # Plot x-y trajectory (spatial path)
    axes[2, 1].plot([val[0] for val in robot_pose_values], [val[1] for val in robot_pose_values], '^-', label="x-y trajectory")
    axes[2, 1].set_title("Robot Pose: X-Y Spatial Path")
    axes[2, 1].set_xlabel("X Position")
    axes[2, 1].set_ylabel("Y Position")
    axes[2, 1].legend()
    axes[2, 1].grid()
    axes[2, 1].set_aspect('equal', adjustable='datalim')

    plt.tight_layout()
    plt.subplots_adjust(hspace=0.4, top=0.9)
    plt.show()


def calculate_performance_metrics(controller_data):
    # Compute agility as 1 / integral of absolute error
    agility = 1.0 / np.trapz(np.abs(controller_data['e']), controller_data['stamp'] * 1e-9)
    
    # Compute accuracy as 1 / absolute value of end state error
    accuracy = 1.0 / np.abs(controller_data['e'].iloc[-1])

    # Compute overshoot as peak error - end state error
    overshoot = np.max(controller_data['e']) - controller_data['e'].iloc[-1]
        
    return agility, accuracy, overshoot


def compare_controllers(p_dir, pid_dir):
    results = []
    
    directories = [p_dir, pid_dir]

    for directory in directories:
        # Controller type based on folder name
        controller_type = os.path.basename(directory)
        
        angular_file = os.path.join(directory, "angular.csv")
        linear_file = os.path.join(directory, "linear.csv")
        
        # Load angular data and compute metrics
        angular_data = pd.DataFrame(FileReader(angular_file).read_file()[1], columns=FileReader(angular_file).read_file()[0])
        angular_metrics = calculate_performance_metrics(angular_data)
        
        # Load linear data and compute metrics
        linear_data = pd.DataFrame(FileReader(linear_file).read_file()[1], columns=FileReader(linear_file).read_file()[0])
        linear_metrics = calculate_performance_metrics(linear_data)
        
        results.append({
            "Controller": controller_type,
            "Angular_Agility": angular_metrics[0],
            "Angular_Accuracy": angular_metrics[1],
            "Angular_Overshoot": angular_metrics[2],
            "Linear_Agility": linear_metrics[0],
            "Linear_Accuracy": linear_metrics[1],
            "Linear_Overshoot": linear_metrics[2]
        })
    
    comparison_table = pd.DataFrame(results)

    print("Controller Comparison Table (Linear and Angular):")
    print(comparison_table)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process some files.")
    parser.add_argument('--files', nargs='+', help='List of controller directories to plot')
    parser.add_argument('--p_dir', help='P controller directory')
    parser.add_argument('--pid_dir', help='PID controller directory')

    args = parser.parse_args()

    # Plot errors for each controller directory provided
    if args.files:
        for directory in args.files:
            print(f"Plotting data from directory: {directory}")
            plot_errors(directory, controller_type=directory.split("/")[-1].upper())
    
    # Compare P and PID controllers if specified
    if args.p_dir and args.pid_dir:
        print("Comparing P and PID controllers based on provided directories.")
        compare_controllers(args.p_dir, args.pid_dir)
