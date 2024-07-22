FROM ubuntu:20.04 as basei

# ROS install
# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \ 
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update 

RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    python3-rosdep

# FROM base as dev

# Development and simulation tools
# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    tmux \
    nano \
    git

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-plugins

# VR
RUN apt-get update && apt-get install -y keyboard-configuration
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    build-essential cmake cppcheck curl git gnupg libeigen3-dev \
    libgles2-mesa-dev lsb-release pkg-config protobuf-compiler \
    qtbase5-dev python3-dbg python3-pip python3-venv ruby software-properties-common wget
    
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-hector-gazebo-plugins \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-key-teleop \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-teleop-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-velodyne-simulator \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins

# When building the image if there is a "403 Forbidden" error running below command, then go to https://foxglove.dev/download and update the download link
RUN wget https://get.foxglove.dev/desktop/latest/foxglove-studio-2.9.0-linux-amd64.deb -O ./foxglove2.9.0.deb \
     && apt install -y ./foxglove2.9.0.deb 

# Load ROS environments    
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/autonomous_catamaran_ws/devel/setup.bash" >> ~/.bashrc

# As this docker container would be used as a client to see the pi's ros topics, them the master is set to the pi's IP address.
RUN echo "#ROS_MASTER_URI=http://pi-desktop:11311" >> ~/.bashrc
RUN echo "ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
# Custom colored prompt
RUN echo "PS1='\\[\\033[01;34m\\] (Docker)\\[\\033[0m\\] \${debian_chroot:+(\$debian_chroot)}\\u@\\h:\\[\\033[38;5;172m\\]\\w\\[\\033[0m\\] \\$ '"  >> ~/.bashrc
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=

#tmux config
RUN echo "set -g mouse on" >> ~/.tmux.conf

# script to install all rosdeps
COPY ./docker/rosdeps.sh /usr/local/bin/rosdeps.sh
## Make the setup script executable
RUN chmod +x /usr/local/bin/rosdeps.sh
#ENTRYPOINT ["/usr/local/bin/rosdeps.sh"]
RUN echo -e "\n\n\033[0;31m Run /usr/local/bin/rosdeps.sh"
