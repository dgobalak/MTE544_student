from math import atan2, asin, sqrt

M_PI = 3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str = ""

            for header in headers:
                header_str += header
                header_str += ", "
            
            header_str += "\n"
            
            file.write(header_str)

    def log_values(self, values_list):
        with open(self.filename, 'a') as file:
            vals_str = ""
            
            for value in values_list:
                vals_str += f"{value}, "
            
            vals_str += "\n"
            
            file.write(vals_str)

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        self.filename = filename
        
    def read_file(self):
        read_headers = False
        table = []
        headers = []

        with open(self.filename, 'r') as file:
            if not read_headers:
                for line in file:
                    values = line.strip().split(',')

                    for val in values:
                        if val == '':
                            break
                        headers.append(val.strip())

                    read_headers = True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row = []                
                for val in values:
                    if val == '':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw

# Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
    """
    Calculate the Euclidean distance between the current_pose and the goal_pose.
    current_pose = [x, y, theta, timestamp]
    goal_pose = [x, y]
    """
    x_current, y_current = current_pose[0], current_pose[1]
    x_goal, y_goal = goal_pose[0], goal_pose[1]

    # Compute the Euclidean distance
    error_linear = sqrt((x_goal - x_current) ** 2 + (y_goal - y_current) ** 2)

    return error_linear

# Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):
    """
    Calculate the angular error between the robot's current orientation and the direction to the goal.
    current_pose = [x, y, theta, timestamp]
    goal_pose = [x, y]
    """
    x_current, y_current, theta_current = current_pose[0], current_pose[1], current_pose[2]
    x_goal, y_goal = goal_pose[0], goal_pose[1]

    # Desired angle to the goal
    theta_goal = atan2(y_goal - y_current, x_goal - x_current)

    # Calculate the angular error
    error_angular = theta_goal - theta_current

    # Ensure that the error is within the range [-pi, pi]
    while error_angular > M_PI:
        error_angular -= 2 * M_PI
    while error_angular < -M_PI:
        error_angular += 2 * M_PI

    return error_angular
