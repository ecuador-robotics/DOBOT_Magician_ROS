ARG ROS2DISTRO=humble
FROM ros:${ROS2DISTRO}

# Redeclare ARG after FROM to make it available in this build stage
ARG ROS2DISTRO=humble

# Install basic dev tools
RUN apt-get update && apt-get install -y \
    ros-${ROS2DISTRO}-desktop \
    git \
    python3-pip \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Add user to video group for hardware acceleration if available
# RUN usermod -aG video ros

# Source ROS setup in bashrc
RUN echo "source /opt/ros/${ROS2DISTRO}/setup.bash" >> /root/.bashrc