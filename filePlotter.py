# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
from math import atan2, asin
import argparse

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()
    plt.show()
    

def plot_imu(filename):
    headers, values=FileReader(filename).read_file()
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)
        
    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    plt.legend()    
    plt.grid()
    plt.show()
    
    
def plot_odom(filename):
    headers, values=FileReader(filename).read_file()
    time_list=[]
    first_stamp=values[0][-1]
    
    # plot x vs y, x/y/yaw vs time in seaprate subplots
    
    for val in values:
        time_list.append(val[-1] - first_stamp)
        
    # declare 1 figure with 2 subplots
    fig, axs = plt.subplots(2)
    
    # plot x vs y
    axs[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axs[0].set_title('x vs y')
    
    # separate the subplots
    plt.tight_layout()
    
    # plot x/y/yaw vs time
    axs[1].plot(time_list, [lin[0] for lin in values], label='x')
    axs[1].plot(time_list, [lin[1] for lin in values], label='y')
    axs[1].plot(time_list, [lin[2] for lin in values], label='theta')
    axs[1].legend()
    axs[1].set_title('x/y/yaw vs time')
    
    plt.grid()
    plt.show()
    
    
def plot_laser(filename):
    pass
    
    
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
    t2 = 1.0 if t2 >  1.0 else t2
    t2 = 1.0 if t2 < -1.0 else t2
    pitch = asin(t2)

    # Yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw

    
if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        if "odom" in filename:
            plot_odom(filename)
        elif "imu" in filename:
            plot_imu(filename)
        elif "laser" in filename:
            plot_laser(filename)
            
        # plot_errors(filename)
