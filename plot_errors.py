import matplotlib.pyplot as plt
from utilities import FileReader
import pandas as pd
import numpy as np


def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)


    
    fig, axes = plt.subplots(2,1, figsize=(14,6))
    
    df = pd.read_csv("commands.csv")
    df['dt'] = df[' ts'].diff().fillna(0)

    velocity_x = [0]  # starting velocity assumed to be 0
    velocity_y = [0]
    position_x = [0]  # starting position assumed to be 0
    position_y = [0]
    theta = [0]

    for i in range(1, len(df)):
        theta.append(theta[-1] + df[' w'].iloc[i] * df['dt'].iloc[i])
        vx = df["v"].iloc[i] * np.cos(theta[i])
        vy = df["v"].iloc[i] * np.sin(theta[i])

        velocity_x.append(vx)
        velocity_y.append(vy)
        
        px = position_x[-1] + vx * df['dt'].iloc[i]
        py = position_y[-1] + vy * df['dt'].iloc[i]

        position_x.append(px)
        position_y.append(py)

    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values])
    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])    
    axes[0].set_title("state space")
    axes[0].grid()
    axes[0].set_aspect('equal', adjustable='box')
    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


