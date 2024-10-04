import os
import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin, isinf, isnan
from utilities import FileReader

def plot_combined_figure(odom_file, imu_file, laser_file, title):
    # Read Odom file
    _, odom_values = FileReader(odom_file).read_file()
    
    # Read IMU file
    _, imu_values = FileReader(imu_file).read_file()

    # Extract time values for each sensor
    odom_time = [val[-1] - odom_values[0][-1] for val in odom_values]
    imu_time = [val[-1] - imu_values[0][-1] for val in imu_values]

    # Set up figure with 4 subplots (2 per row)
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))

    # First row, left: x vs y from Odom
    axs[0, 0].plot([lin[0] for lin in odom_values], [lin[1] for lin in odom_values], label='x vs y')
    axs[0, 0].set_title('x vs y')
    axs[0, 0].set_xlabel('x (meters)')
    axs[0, 0].set_ylabel('y (meters)')
    axs[0, 0].legend()

    # First row, right: x, y, and theta from Odom
    ax1 = axs[0, 1]
    ax2 = ax1.twinx()
    ax1.plot(odom_time, [lin[0] for lin in odom_values], 'g-', label='x')
    ax1.plot(odom_time, [lin[1] for lin in odom_values], 'b-', label='y')
    ax2.plot(odom_time, [lin[2] for lin in odom_values], 'r-', label='theta')

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (meters)')
    ax2.set_ylabel('Theta (radians)')
    ax1.legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax1.set_title('x, y, and theta vs time')

    # Second row, left: acc_x, acc_y, and angular_z from IMU
    ax3 = axs[1, 0]
    ax4 = ax3.twinx()
    ax3.plot(imu_time, [lin[0] for lin in imu_values], 'g-', label='acc_x')
    ax3.plot(imu_time, [lin[1] for lin in imu_values], 'b-', label='acc_y')
    ax4.plot(imu_time, [lin[2] for lin in imu_values], 'r-', label='angular_z')

    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (m/s²)')
    ax4.set_ylabel('Angular Velocity (rad/s)')
    ax3.legend(loc="upper left")
    ax4.legend(loc="upper right")
    ax3.set_title('acc_x, acc_y, and angular_z vs time')

    # Second row, right: Lidar data from all rows of laser file
    # Note that the data in the CSV is stored in an array string format, so we need to evaluate it without FileReader
    with open(laser_file, 'r') as file:
        lines = file.readlines()

    # Set up the color map and color bar for the Lidar scans over time
    cmap = plt.get_cmap("viridis")
    num_rows = len(lines) - 1  # Number of Lidar scans (skip header)
    
    norm = matplotlib.colors.Normalize(vmin=0, vmax=num_rows)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])

    for i in range(1, num_rows):
        # Parse the LIDAR data
        data_line = lines[i].strip()
        start = data_line.find("array('f', [") + len("array('f', [")
        end = data_line.find("])")
        ranges_str = data_line[start:end].replace("inf", "float('inf')")
        ranges = eval(f"[{ranges_str}]")

        rest_of_line = data_line[end+2:].strip()  # Everything after "])"
        angle_increment, timestamp = rest_of_line.split(',')[1:3]

        # Extract angle_increment and timestamp
        angle_increment = float(angle_increment.strip())
        timestamp = timestamp.strip()

        # Convert ranges to x, y coordinates using the angle at which the range was measured
        angles = np.arange(len(ranges)) * angle_increment
        x_lidar = []
        y_lidar = []
        
        for r, a in zip(ranges, angles):
            if not isinf(r) and not isnan(r):
                x_lidar.append(r * cos(a))
                y_lidar.append(r * sin(a))

        # Plot each row with a color from the colormap
        color = cmap(i / num_rows)
        axs[1, 1].scatter(x_lidar, y_lidar, s=1, color=color)

    axs[1, 1].set_title(f'LIDAR scans over time')
    axs[1, 1].set_xlabel('x (meters)')
    axs[1, 1].set_ylabel('y (meters)')
    axs[1, 1].grid(True)

    # Add a color bar to represent time progression
    cbar = fig.colorbar(sm, ax=axs[1, 1], orientation='vertical')
    cbar.set_label('Lidar Scan Time (row index)')

    # Set the title and adjust the layout
    fig.suptitle(title)
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    
    # Save the figure to the same directory as the CSV files
    plt.savefig(f"{os.path.dirname(odom_file)}/{title}.png")
    # plt.show()
    

def process_directory(directory):
    # Assume that the directory contains CSV files for each shape (circle, line, spiral)
    
    # Group csv files by shape and sensor type (odom, imu, laser)
    files_grouped = {
        "circle": {},
        "line": {},
        "spiral": {}
    }

    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        if filename.endswith(".csv"):
            if "odom" in filename:
                if "circle" in filename:
                    files_grouped["circle"]["odom"] = file_path
                elif "line" in filename:
                    files_grouped["line"]["odom"] = file_path
                elif "spiral" in filename:
                    files_grouped["spiral"]["odom"] = file_path
            elif "imu" in filename:
                if "circle" in filename:
                    files_grouped["circle"]["imu"] = file_path
                elif "line" in filename:
                    files_grouped["line"]["imu"] = file_path
                elif "spiral" in filename:
                    files_grouped["spiral"]["imu"] = file_path
            elif "laser" in filename:
                if "circle" in filename:
                    files_grouped["circle"]["laser"] = file_path
                elif "line" in filename:
                    files_grouped["line"]["laser"] = file_path
                elif "spiral" in filename:
                    files_grouped["spiral"]["laser"] = file_path

    # Now that we have grouped files, plot for each motion type (circle, line, spiral)
    for shape in files_grouped:
        if all(key in files_grouped[shape] for key in ("odom", "imu", "laser")):
            plot_combined_figure(files_grouped[shape]["odom"], files_grouped[shape]["imu"], files_grouped[shape]["laser"], f"{shape.capitalize()} Data")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Process all CSV files in a directory.')
    parser.add_argument('--dir', required=True, help='Directory containing the CSV files to process')

    args = parser.parse_args()

    # Process all CSV files in the given directory
    process_directory(args.dir)
