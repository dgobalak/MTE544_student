import numpy as np
from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI = 3.1415926535

# Controller types
P = 0; PD = 1; PI = 2; PID = 3

class controller:
    
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        # TODO Set up the controllers for linear and angular velocity
        self.PID_linear = PID_ctrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular = PID_ctrl(P, kap, kav, kai, filename_="angular.csv")

    def vel_request(self, pose, goal, status):
        # Calculate errors
        e_lin = calculate_linear_error(pose, goal)
        e_ang = calculate_angular_error(pose, goal)

        # Compute control inputs using the P-controller
        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)
        
        # Saturation limits for the robot's linear and angular velocities
        # Real robot
        max_linear_vel = 0.31
        max_angular_vel = 1.9
        
        # Simulation
        # max_linear_vel = 0.22
        # max_angular_vel = 2.84
        
        # Apply saturation limits
        linear_vel = min(max(linear_vel, -max_linear_vel), max_linear_vel)
        angular_vel = min(max(angular_vel, -max_angular_vel), max_angular_vel)
        
        return linear_vel, angular_vel

class trajectoryController(controller):
    
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        goal = self.lookFarFor(pose, listGoals)
        finalGoal = listGoals[-1]
        
        e_lin = calculate_linear_error(pose, finalGoal)
        e_ang = calculate_angular_error(pose, goal)

        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # Real robot
        max_linear_vel = 0.31
        max_angular_vel = 1.9
        
        # Simulation
        # max_linear_vel = 0.22
        # max_angular_vel = 2.84
        
        linear_vel = min(max(linear_vel, -max_linear_vel), max_linear_vel)
        angular_vel = min(max(angular_vel, -max_angular_vel), max_angular_vel)
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        poseArray = np.array([pose[0], pose[1]]) 
        listGoalsArray = np.array(listGoals)
        distanceSquared = np.sum((listGoalsArray - poseArray) ** 2, axis=1)
        closestIndex = np.argmin(distanceSquared)

        return listGoals[min(closestIndex + 3, len(listGoals) - 1)]
