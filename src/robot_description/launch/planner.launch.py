from launch import  LaunchDescription
from launch_ros.actions import Node
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{'yaml_filename': '/home/varun/maps/warehouse_map.yaml'}]
    )

    global_planner_node = Node(
        package="my_robot_planner",
        executable="global_planner_node.py",
        name="global_planner_node",
        output="screen"
    ) 

    local_planner_node = Node(
        package="my_robot_planner",
        executable="local_planner_node.py",
        name="local_planner_node",
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    # turtlebot4_ignition_bringup_node= Node(
    #     package="turtlebot4_ignition_bringup",
    #     executable="turtlebot4_ignition.launch.py",
    #     output="screen"

    # )

    turtlebot4_ignition_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/varun/turtlebot4_ws/install/turtlebot4_ignition_bringup/share/turtlebot4_ignition_bringup/launch',
                'turtlebot4_ignition.launch.py'

            )
        )
    )

    return LaunchDescription([
        map_server_node,
        global_planner_node,
        local_planner_node,
        rviz2_node,
        turtlebot4_ignition_bringup_node
    ])