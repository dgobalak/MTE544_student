import os
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, asin, cos, sin, isinf, isnan
import argparse
from utilities import FileReader
import ast

def plot_imu(filename):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label=headers[i])

    # Set title of the plot to be the filename
    plt.title(filename)
    plt.xlabel("Time (s)")
    plt.ylabel("IMU Values")
    plt.legend()
    plt.grid()
    plt.show()


def plot_odom(filename):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    # Plot x vs y, and x/y/yaw vs time in separate subplots
    for val in values:
        time_list.append(val[-1] - first_stamp)

    # Declare 1 figure with 2 subplots: one for x/y, one for x/y/yaw vs time
    fig, axs = plt.subplots(3, figsize=(8, 6))

    # Plot x vs y
    axs[0].plot([lin[0] for lin in values], [lin[1] for lin in values], label='x vs y')
    axs[0].set_title('x vs y')
    axs[0].set_xlabel('x (meters)')
    axs[0].set_ylabel('y (meters)')
    axs[0].legend()

    # Plot x/y vs time
    axs[1].plot(time_list, [lin[0] for lin in values], label='x')
    axs[1].plot(time_list, [lin[1] for lin in values], label='y')
    axs[1].set_title('x/y/theta vs time')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Position (meters)')
    axs[1].legend()
    
    # plot theta vs time
    axs[2].plot(time_list, [lin[2] for lin in values], label='theta')
    axs[2].set_title('theta vs time')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('theta (radians)')
    axs[2].legend()

    # Set the overall title of the plot to be the filename
    fig.suptitle(filename)

    # Add space under title
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)

    plt.grid()
    plt.show()


def plot_first_laser_row(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    # Skip the header (first line)
    data_line = lines[1].strip()  # Take the first data row (second line)

    # Find the starting point of the ranges, which is within "array('f', [...])"
    start = data_line.find("array('f', [") + len("array('f', [")
    end = data_line.find("])")
    
    # Extract the ranges string and convert it to a list of floats
    ranges_str = data_line[start:end].replace("inf", "float('inf')")
    ranges = eval(f"[{ranges_str}]")  # Convert string to list of floats

    # Extract the angle_increment and timestamp (which are after the array)
    rest_of_line = data_line[end+2:].strip()  # Everything after "])"
    angle_increment, timestamp = rest_of_line.split(',')[-2:]

    # Convert them to appropriate types
    angle_increment = float(angle_increment.strip())
    timestamp = timestamp.strip()

    # Convert ranges and angle_increment to Cartesian coordinates
    angles = np.arange(len(ranges)) * angle_increment
    x = []
    y = []

    # Clean NaN and Inf values and convert to Cartesian coordinates
    for r, a in zip(ranges, angles):
        if not isinf(r) and not isnan(r):
            x.append(r * cos(a))
            y.append(r * sin(a))

    # Plot the LIDAR data
    plt.scatter(x, y, s=1)  # Small point size for LIDAR data
    plt.title(f'LIDAR scan from {filename} at {timestamp}')
    plt.xlabel('x (meters)')
    plt.ylabel('y (meters)')
    plt.grid(True)
    plt.show()


def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """

    # Roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)

    # Yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw


def process_directory(directory):
    # List all CSV files in the given directory
    for filename in os.listdir(directory):
        if filename.endswith(".csv"):
            file_path = os.path.join(directory, filename)
            print(f"Processing file: {file_path}")

            # Determine which plot function to call based on the filename
            if "odom" in filename:
                plot_odom(file_path)
            elif "imu" in filename:
                plot_imu(file_path)
            elif "laser" in filename:
                plot_first_laser_row(file_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process all CSV files in a directory.')
    parser.add_argument('--dir', required=True, help='Directory containing the CSV files to process')

    args = parser.parse_args()

    # Call the function to process the directory
    process_directory(args.dir)
