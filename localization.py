import sys
from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0

class localization(Node):
    
    def __init__(self, localizationType=rawSensor):
        super().__init__("localizer")
        
        # Part 3: Define the QoS profile based on simulation or real robot
        odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)
        
        self.loc_logger = Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose = None
        
        if localizationType == rawSensor:
            # Part 3: Subscribe to the position sensor topic (Odometry)
            self.subscription = self.create_subscription(odom, '/odom', self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    def odom_callback(self, pose_msg):
        # Part 3: Read x, y, theta, and record the stamp
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        
        # Convert quaternion to Euler angles
        orientation_q = pose_msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Record the timestamp and log the pose
        timestamp = Time.from_msg(pose_msg.header.stamp).nanoseconds
        self.pose = [x, y, theta, timestamp]
        self.loc_logger.log_values([x, y, theta, timestamp])
    
    def getPose(self):
        return self.pose

# Part 3: Guard to run the node only when executed directly
if __name__ == "__main__":
    init()
    loc_node = localization(rawSensor)
    
    try:
        spin(loc_node)
    except KeyboardInterrupt:
        print("Shutting down the localization node")
        loc_node.loc_logger.save_log()
        loc_node.destroy_node()
        sys.exit(0)
