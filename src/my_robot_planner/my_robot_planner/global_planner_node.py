#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped
# # from a_star import AStarPlanner
# import numpy as np
# from my_robot_planner.a_star import AStarPlanner
 
# class GlobalPlannerNode(Node):
#     def __init__(self):
#         super().__init__('global_planner')
#         self.grid_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
#         self.goal_sub = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
#         self.path_pub = self.create_publisher(Path, 'planned_path', 10)
#         self.get_logger().info("Hey. Global Planner Node has been started...")
#         self.grid = None
#         self.resolution = 0.05

#     def map_callback(self, msg):
#         self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
#         self.resolution = msg.info.resolution

#     def goal_callback(self, msg):
#         if self.grid is None:
#             self.get_logger().error('Map is not yet available!')
#             return

#         start = (0, 0)  # Replace with robot's position on the grid
#         goal = (
#             int(msg.pose.position.x / self.resolution),
#             int(msg.pose.position.y / self.resolution)
#         )

#         planner = AStarPlanner(self.grid, self.resolution)
#         path = planner.plan(start, goal)

#         if path:
#             ros_path = Path()
#             ros_path.header.frame_id = "map"
#             for p in path:
#                 pose = PoseStamped()
#                 pose.header.frame_id = "map"
#                 pose.pose.position.x = p[0] * self.resolution
#                 pose.pose.position.y = p[1] * self.resolution
#                 ros_path.poses.append(pose)
#             self.path_pub.publish(ros_path)
#         else:
#             self.get_logger().error('No path found!')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GlobalPlannerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# scripts/global_planner_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from a_star import a_star

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.get_logger().info("Global planner node have been started")
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)
        self.publisher = self.create_publisher(PoseArray, 'global_path', 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.map_data = None
        self.start_pose = (0, 0)
        self.goal_pose = None

    def map_callback(self, msg):
        self.map_data = msg.data

    def goal_pose_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        if self.map_data:
            self.plan_path()

    def plan_path(self):
        grid = self.convert_map_to_grid(self.map_data)
        path = a_star(self.start_pose, self.goal_pose, grid)
        if path:
            self.publish_path(path)

    def convert_map_to_grid(self, map_data):
        # Convert OccupancyGrid data to a 2D grid
        pass

    def publish_path(self, path):
        pose_array = PoseArray()
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose_array.poses.append(pose)
        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()