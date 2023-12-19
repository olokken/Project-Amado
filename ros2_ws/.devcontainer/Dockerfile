# Use an official ROS 2 base image
FROM osrf/ros:humble-desktop-full

# Install any additional packages you might need
RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
ENV ROS_WS=/opt/ros_ws
ENV DISPLAY=novnc:0.0

RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Source the workspace
RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to execute
# when creating a new container
CMD ["bash"]
