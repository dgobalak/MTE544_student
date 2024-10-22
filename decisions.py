# Imports
import sys
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor
from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):
        super().__init__("decision_maker")

        # Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period = 1 / rate
        
        # Part 5: Tune your parameters here
        if motion_type == POINT_PLANNER:
            self.controller = controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner = planner(POINT_PLANNER)    
        elif motion_type == TRAJECTORY_PLANNER:
            self.controller = trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner = planner(TRAJECTORY_PLANNER)
        else:
            print("Error! You don't have this planner", file=sys.stderr)

        # Instantiate the localization, use rawSensor for now  
        self.localizer = localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal = self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)

    def timerCallback(self):
        # Part 3: Run the localization node
        spin_once(self.localizer)

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg = Twist()
        
        # Part 3: Check if you reached the goal
        if type(self.goal) == list:
            # Trajectory planner
            reached_goal = calculate_linear_error(self.localizer.getPose(), self.goal[-1]) < 0.1
        else: 
            # point planner
            reached_goal = calculate_linear_error(self.localizer.getPose(), self.goal) < 0.1
        
        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            # Part 4: Exit the node spin
            self.get_logger().info("reached goal")
            self.get_logger().info("shutting down the node")
            self.destroy_node()
            sys.exit(0)
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)

import argparse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def main(args=None):
    init()

    # Part 3: You might need to change the QoS profile based on the robot or simulation
    # For real robot
    odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)
    
    # For sim
    # odom_qos = QoSProfile(
    #     reliability=QoSReliabilityPolicy.RELIABLE,
    #     history=QoSHistoryPolicy.KEEP_LAST,
    #     depth=10
    # )
    
    # Part 4: Instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM = decision_maker(publisher_msg=Twist, publishing_topic="/cmd_vel", qos_publisher=odom_qos, goalPoint=[1, 1], motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(publisher_msg=Twist, publishing_topic="/cmd_vel", qos_publisher=odom_qos, goalPoint=None, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)

    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__ == "__main__":
    argParser = argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
