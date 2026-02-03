FROM osrf/ros:humble-desktop-full

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create non-root user if not exists
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Install sudo
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to user
USER $USERNAME

# Allow video access (VNC / GUI safety)
RUN sudo usermod --append --groups video $USERNAME

# System update
RUN sudo apt update && sudo apt upgrade -y

# Basic tools
RUN sudo apt install -y \
    git \
    wget \
    curl \
    lsb-release \
    software-properties-common

# rosdep
RUN rosdep update

# Source ROS automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# MoveIt + ros2_control
RUN sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control
