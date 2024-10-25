import numpy as np

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # parabola for x in [0, 1.5]
        return [[x, x**2] for x in np.linspace(0.0, 1.0, 20)]
        
        # Uncomment when you want to run the sigmoid trajectory
        # sigmoid for x in [0, 2.5]
        # return [[x, 2 / (1 + np.exp(-2*x)) - 1] for x in np.linspace(0, 2.5, 20)]
