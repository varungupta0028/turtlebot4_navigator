
from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='my_robot_planner',
#             executable='global_planner_node',
#             name='global_planner_node',
#             output='screen',
#         ),
#     ])

def generate_launch_description():
    ld = LaunchDescription()

    global_planner_node = Node(
        package="my_robot_planner",
        executable="global_planner_node.py",
        name="global_planner_node",
        output="screen"
    )    
    
    ld.add_action(global_planner_node)

    return ld