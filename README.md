## Mini-BREAM

This repository contains the documentation and source code for deploying an autonomous path planning system on the Mini-BREAM (Biodiversity Research Environmental Assessment Monohull) ASV (Autonomous Surface Vehicle) platform. The system implements Dubins path planning for autonomous navigation using ROS1 Noetic, with development done using Docker under Linux.

### System Requirements
- Ubuntu 22.04 or higher
- Docker Compose v2.24.2 or higher
- Docker Engine v25 or higher

## Project Directory Structure

```
MINI_BREAM
├── docs/                           # Documentation files
│   ├── deployment.md              # Deployment guide for the actual vehicle
│   ├── ground_station_setup.md    # Setting up ground station for monitoring
│   └── port_to_new_rpi.md        # Guide for setting up a new Raspberry Pi
├── src/
│   ├── autonomous_catamaran_ws/   # ROS1 workspace
│   │   └── src/
│   │       ├── backseat/          # Path planning and control logic
│   │       ├── frontseat/         # Low-level vehicle control and sensor drivers
│   │       ├── mission_planner/   # Mission management and file processing
│   │       └── visualization_tools/ # RViz configurations and visualization tools
│   └── docker/                    # Docker configuration
│       ├── Dockerfile
│       ├── docker-compose.yml
│       └── rosdeps.sh
├── .gitignore
├── README.md
└── sync_ws.sh                     # Script for syncing code to Raspberry Pi
```

### ROS Packages Overview

- **backseat**: High-level autonomy package containing path planning algorithms (Dubins paths), mission execution logic, and action server for waypoint navigation.
- **frontseat**: Low-level control package that interfaces with hardware sensors (GPS, IMU) and actuators (motors). Handles sensor data processing and motor control.
- **mission_planner**: Mission management package that processes mission files from various formats (CSV, GPS coordinates, XY coordinates) and sends them to the path planner.
- **visualization_tools**: Provides RViz configuration files and Python scripts for visualizing vehicle state, trajectories, and mission waypoints.
- **sync_ws.sh**: Shell script for synchronizing the workspace to the Mini-BREAM Raspberry Pi when developing and testing with physical sensors.


## Setup and Installation

### 1. Build the Docker Image and Run the Container

Build and start the Docker container:
```shell
cd src/docker
docker compose up -d --build
```

> **Note**: If you encounter a "403 Forbidden" error during the build, visit https://foxglove.dev/download and update the Foxglove download link in the Dockerfile (line 65).

### 2. Access the Container

Enable GUI support and log into the container:
```shell
xhost +local:docker  # Allows GUI applications to display
docker exec -it dev /bin/bash
```

Once inside, you'll be at `/workspace/autonomous_catamaran_ws`. The `/workspace` directory is mounted from your host's `mini_bream/src` folder, so changes are synchronized bidirectionally.

### 3. Install VRX Simulator (For Simulation Only)

The VRX simulator is required for testing in simulation:
```shell
cd /workspace/autonomous_catamaran_ws/src
git clone -b gazebo_classic git@github.com:osrf/vrx.git
```

> **Note**: If the clone fails inside Docker, try cloning from your host machine in the `autonomous_catamaran_ws/src` folder.

### 4. Build the ROS Workspace

Build the workspace and install dependencies:
```shell
cd /workspace/autonomous_catamaran_ws
catkin_make
source devel/setup.bash

# Install ROS dependencies
cd /workspace/docker
./rosdeps.sh

# Install Python dependencies
pip install pandas==1.2 utm scipy
```

> **TODO**: Python dependencies should be added to the Dockerfile or package dependencies.

## Usage

For detailed instructions on running missions in simulation or on the actual vehicle, see the [Getting Started Guide](src/autonomous_catamaran_ws/src/README.md).

For deployment on the physical Mini-BREAM vehicle, refer to the [Deployment Guide](docs/deployment.md).


## Additional Documentation

- **[Getting Started Guide](src/autonomous_catamaran_ws/src/README.md)**: Instructions for running sample missions in simulation
- **[Deployment Guide](docs/deployment.md)**: Steps for deploying on the physical Mini-BREAM vehicle
- **[Ground Station Setup](docs/ground_station_setup.md)**: Configuring a ground station for real-time monitoring
- **[Raspberry Pi Setup](docs/port_to_new_rpi.md)**: Complete guide for setting up a new Raspberry Pi

## Known Issues

- **Action Server**: When setting a new goal for the action server, it sometimes doesn't process the goal but still returns a result, causing the `result_cbk` on the action client to execute. If the mission trajectory doesn't appear in RViz or `Mission_complete=False` appears in the logs, try re-running the launch file.

## Future Improvements

- **Mission Format Standardization**: The PathPlannerNode should accept only one standardized mission format (list of waypoints). Each client (CSV loader, Neptus, RViz custom point) should be responsible for converting their data to this standard format, eliminating the need for format-specific processing in the PathPlannerNode.
- **Python Dependencies**: Move Python dependencies from manual installation to Dockerfile or package dependencies.