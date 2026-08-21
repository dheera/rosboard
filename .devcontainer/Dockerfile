FROM ros:jazzy

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install common development tools and ROS2 development packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    wget \
    curl \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create ros user with sudo privileges
ARG USERNAME=vscode
ARG USER_UID=1100
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment for the ros user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/ros/jazzy/setup.zsh 2>/dev/null || true" >> /home/$USERNAME/.zshrc || true

# Set the default user
USER $USERNAME
WORKDIR /workspaces/rosboard

# Set up environment
ENV SHELL=/bin/bash
