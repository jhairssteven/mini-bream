## Mini-bream
This is the documentation and source code to deploy the Dubins path planner on the Mini-BREAM ASV platform. It uses ROS2 humble and development is done using docker under linux. 

### System requirements
- Ubuntu +20.04
- Docker compose +v2.24.2
- Docker engine +25

## Project Directory Structure

Here is a structure overview of the directories in this repository:

```
MINI_BREAM
├── docs
├── src
│   ├── autonomous_catamaran_ws
│   └── docker
├── .gitignore
├── README.md
└── sync_ws.sh
```

- **docs**: Directory containing documentation for the project.
- **src**: Source code directory.
  - **autonomous_catamaran_ws**: ROS1 Workspace for the autonomous catamaran project.
  - **docker**: Docker configuration and related files.
- **sync_ws.sh**: Shell script for synchronizing the workspace to the mini_bream RPi when developing and testing with sensors.


### Clone the repo 
```shell
git clone git@github.com:jhairssteven/mini-bream.git
cd mini-bream/
git submodule update --init --recursive
```

### Build the docker image and run the container

1. Build the docker image with
```shell
cd src/docker
docker compose up -d --build
```

2. Log into the container with
```shell
xhost +local:docker # this allows the graphics
docker exec -it dubins_planner /bin/bash
```

Once you execute the command you should be logged on the container at path the `/workspace` folder, which is a direct link to the folder `mini_bream/src` of your local host, any changes made to files on this folder within the container would be reflected on your machine (and viceversa).

<!-- > **Note: Every command below should be executed inside the docker container** -->

4. Build the workspace with
```shell
cd /workspace/vrx_ws
colcon build --merge-install
source install/setup.bash

cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Next steps
**Reference to the [Getting started](Getting%20Started.md) guide for usage**.


## To-dos
- The format of the mission should be a list of Waypoints, when loading a new mission, we don't need to process and adjust each incoming format on the PathPlannerNode. The PathPlannerNode should accept only one format. Each client should then be responsible of formatting its data so that the PathPlanner node can understand it. For example, if loading from a csv file, or from neptus or sending a custom point from Rviz. Those could be three different clients and no changes should be made to account for this compatibility on the PathPlannerNode.

## Issues
- When setting a new goal for the action server, sometimes the server doesn't process it, but still returns a result and the result_cbk on the actionClient gets executed..