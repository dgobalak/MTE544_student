
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import yaml

# Fetch robot odom data
robot_pose_df = pd.read_csv('data/euc2/robotPose.csv')
robot_pose_df.columns = robot_pose_df.columns.str.strip()

# Fetch map
map_image_path = 'room.pgm'
map_image = Image.open(map_image_path)
map_array = np.array(map_image)

yaml_path = 'room.yaml'
with open(yaml_path, 'r') as file:
    map_metadata = yaml.safe_load(file)

resolution = map_metadata['resolution']
origin = map_metadata['origin']

# Transform robot trajectory to map pixel coordinates
def world_to_map(x, y, resolution, origin):
    map_x = int((x - origin[0]) / resolution)
    map_y = int((y - origin[1]) / resolution)
    return map_x, map_array.shape[0] - map_y  # Flip y-axis for image coordinates

trajectory_pixels = [
    world_to_map(row['x'], row['y'], resolution, origin) for _, row in robot_pose_df.iterrows()
]

waypoints = []
positions = robot_pose_df[['x', 'y']].values
timestamps = robot_pose_df['ts'].values

# Detect points where the robot stops moving for a while
movement_threshold = 0.03 # 3 cm
time_threshold = 2e9  # 5 seconds
for i in range(len(positions)):
    for j in range(i + 1, len(positions)):
        time_diff = timestamps[j] - timestamps[i]
        if time_diff > time_threshold:
            distance = np.linalg.norm(positions[i] - positions[j])
            if distance < movement_threshold:
                waypoints.append(world_to_map(*positions[i], resolution, origin))
                break

# Remove duplicate waypoints
unique_waypoints = []
for i, wp in enumerate(waypoints):
    if i == 0 or wp != waypoints[i - 1]:
        unique_waypoints.append(wp)

# Plot map
plt.figure(figsize=(10, 8))
plt.imshow(map_array, cmap='gray', origin='upper')

# Plot robot trajectory
trajectory_pixels = np.array(trajectory_pixels)
plt.plot(trajectory_pixels[:, 0], trajectory_pixels[:, 1], color='red', linewidth=2, label='Robot Path')

# Plot waypoints
for idx, wp in enumerate(unique_waypoints):
    if idx == 0:  # Start point
        plt.scatter(wp[0], wp[1], color='green', s=100, label='Start')
    else:
        plt.scatter(wp[0], wp[1], color='blue', s=100, label='Waypoint' if idx == 1 else None)

plt.title('Robot Trajectory with Waypoints (Euclidean Heuristic)')
plt.legend()
plt.axis('off')
plt.show()
