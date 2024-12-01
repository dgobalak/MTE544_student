
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        self.m_utilites=mapManipulator(laser_sig=0.4)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        pixelPath = search(self.costMap, startPose, endPose, heuristic_method="manhattan")
        Path = list(map(lambda x: self.m_utilites.cell_2_position(x), pixelPath))



        # TODO PART 5 return the path as list of [x,y]
        return Path



if __name__=="__main__":
    import matplotlib.pyplot as plt
    
    rclpy.init()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    m_utilites=mapManipulator(laser_sig=0.4)
    costMap=m_utilites.make_likelihood_field()

    plt.figure(figsize=(10, 8))
    plt.imshow(costMap, cmap='viridis', alpha=costMap)
    plt.colorbar(label='Grid Intensity')

    plt.title('Grid')
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.grid(color='white', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    plt.show()
    