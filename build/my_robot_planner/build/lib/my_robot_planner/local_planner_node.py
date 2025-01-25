import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        self.get_logger().info("Local planner node has been started...")
        
        # Subscriptions
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Internal variables
        self.current_path = []
        self.obstacle_detected = False
        self.obstacle_threshold = 0.5  # Distance threshold to consider obstacles (meters)
        self.current_index = 0
        
        # Control parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s

    def path_callback(self, msg):
        # Extract path points
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_index = 0

    def scan_callback(self, msg):
        # Check for obstacles in the scan data
        self.obstacle_detected = any(distance < self.obstacle_threshold for distance in msg.ranges if distance > 0.0)

    def follow_path(self):
        if not self.current_path or self.current_index >= len(self.current_path):
            self.publish_stop()
            return

        # Get the current goal point
        goal_x, goal_y = self.current_path[self.current_index]

        # Replace with the robot's current position and orientation (for simplicity, assumed as 0,0,0 here)
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0  # TODO: Replace with localization data

        # Compute control signals
        angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
        distance_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

        if self.obstacle_detected:
            # If an obstacle is detected, stop or take evasive action
            self.get_logger().info('Obstacle detected! Stopping.')
            self.publish_stop()
            return

        twist = Twist()
        if distance_to_goal > 0.1:
            # Move toward the goal
            twist.linear.x = min(self.linear_speed, distance_to_goal)
            twist.angular.z = 2.0 * (angle_to_goal - robot_theta)
        else:
            # Move to the next path point
            self.current_index += 1

        # Publish velocity command
        self.cmd_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    
    # Run the node with a loop to follow the path
    rate = node.create_rate(10)  # 10 Hz
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.follow_path()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()