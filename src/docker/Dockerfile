FROM osrf/ros:humble-desktop

# Set shell to bash
SHELL ["/bin/bash", "-c"]
# Do not prompt the user for input during pkg installation
ENV DEBIAN_FRONTEND=noninteractive

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install ros2 build tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden
RUN apt-get update \
    && apt-get install -y lsb-release curl gnupg

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-garden

## Development tools configs
RUN apt-get update && apt-get install -y \
    tmux \
    nano \
    git

#tmux config
RUN echo "set -g mouse on" >> ~/.tmux.conf

# Custom colored prompt
RUN echo "PS1='\\[\\033[01;34m\\] (Docker)\\[\\033[0m\\] \${debian_chroot:+(\$debian_chroot)}\\u@\\h:\\[\\033[38;5;172m\\]\\w\\[\\033[0m\\] \\$ '"  >> ~/.bashrc

# Switch back to dialog for any user-input interaction when using apt-get
ENV DEBIAN_FRONTEND=dialog
# Set entry point
CMD ["bash"]