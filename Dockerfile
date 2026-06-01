# Vehicle Simulation Workspace Docker Image
# Build:  docker build -t vehicle-sim:humble .
# Run:    docker run -it --net=host --gpus all vehicle-sim:humble

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ---- Install dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 packages
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    # System tools
    python3-pip \
    python3-tk \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# ---- Create workspace ----
RUN mkdir -p /workspace/src
WORKDIR /workspace

# ---- Copy source code ----
COPY src/ /workspace/src/

# ---- Build ----
RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install \
    && echo "source /workspace/install/setup.bash" >> ~/.bashrc

# ---- Entrypoint ----
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
