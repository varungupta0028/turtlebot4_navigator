#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
# from a_star import AStarPlanner
import numpy as np
from my_robot_planner.a_star import AStarPlanner
 
class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.grid_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.get_logger().info("Hey. Global Planner Node has been started...")
        self.grid = None
        self.resolution = 0.05

    def map_callback(self, msg):
        self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution

    def goal_callback(self, msg):
        if self.grid is None:
            self.get_logger().error('Map is not yet available!')
            return

        start = (0, 0)  # Replace with robot's position on the grid
        goal = (
            int(msg.pose.position.x / self.resolution),
            int(msg.pose.position.y / self.resolution)
        )

        planner = AStarPlanner(self.grid, self.resolution)
        path = planner.plan(start, goal)

        if path:
            ros_path = Path()
            ros_path.header.frame_id = "map"
            for p in path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = p[0] * self.resolution
                pose.pose.position.y = p[1] * self.resolution
                ros_path.poses.append(pose)
            self.path_pub.publish(ros_path)
        else:
            self.get_logger().error('No path found!')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
