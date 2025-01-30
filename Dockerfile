# Use the official ROS 2 Humble base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0

# Update and install required dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rclpy \
    ros-humble-nav2-bringup \
    ros-humble-nav2-core \
    ros-humble-nav2-msgs \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-gazebo-ros \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace inside the container
WORKDIR /ros2_ws/src

# Copy the entire workspace (assuming your package is in 'src/')
COPY . /ros2_ws/src/

# Set up ROS 2 workspace
WORKDIR /ros2_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the environment and launch the navigation stack
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\""]
CMD ["ros2", "launch", "turtlebot4_navigator", "navigator.launch.py"]
