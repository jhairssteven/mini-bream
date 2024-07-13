## Mini-bream
This is the documentation and source code to deploy the Dubins path planner on the Mini-BREAM ASV platform. It uses ROS1 noetic and development is done using docker under linux.

### System requirements
- Ubuntu +22.04
- Docker compose +v2.24.2
- Docker engine +25

### Build the docker image and run the container

1. Build the docker image with
```shell
cd src/docker
docker compose up -d --build
```
2. Log into the container with
```shell
xhost +local:docker # this allows the graphics
docker exec -it dev /bin/bash
```

Once you execute the command you should be logged on the container at path `/workspace/autonomous_catamaran_ws`. The `/workspace` folder is a direct link to the folder `mini_bream/src` of your local host, any changes made to files on this folder within the container would be reflected on your machine (and viceversa).

<!-- > **Note: Every command below should be executed inside the docker container** -->

3. Download VRX simulator
```shell
cd /workspace/autonomous_catamaran_ws/src
git clone -b gazebo_classic git@github.com:osrf/vrx.git
```

4. Build the workspace with
```shell
cd /workspace/autonomous_catamaran_ws
catkin_make
source devel/setup.bash
# Install workspace dependencies
cd /workspace/docker
./rosdeps.sh

pip install pandas==1.2 utm scipy # TODO: This should be on the dockerfile description or the pkg dependencies
```

**Reference to the [Getting started](src/autonomous_catamaran_ws/src/README.md) guide for usage**


## To-dos
The format of the mission should be a list of Waypoints, when loading a new mission, we don't need to process and adjust each incoming format on the PathPlannerNode. The PathPlannerNode should accept only one format. Each client should then be responsible of formatting its data so that the PathPlanner node can understand it. For example, if loading from a csv file, or from neptus or sending a custom point from Rviz. Those could be three different clients and no changes should be made to account for this compatibility on the PathPlannerNode.
