FROM osrf/ros:humble-desktop

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

## Development tools and user configs

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    tmux \
    nano \
    git

# Custom colored prompt
RUN echo "PS1='\\[\\033[01;34m\\] (Docker)\\[\\033[0m\\] \${debian_chroot:+(\$debian_chroot)}\\u@\\h:\\[\\033[38;5;172m\\]\\w\\[\\033[0m\\] \\$ '"  >> ~/.bashrc
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=

#tmux config
RUN echo "set -g mouse on" >> ~/.tmux.conf

# Set entry point
CMD ["bash"]