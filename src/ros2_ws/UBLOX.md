# ublox in this workspace

GPS drivers and messages come from **apt**, not the git submodule:

```bash
sudo apt install ros-humble-ublox-msgs ros-humble-ublox-gps
```

The `src/ublox` submodule is optional reference only. Building it from source requires
`libasio-dev` and duplicates the apt packages. When building the workspace, skip it:

```bash
colcon build --packages-ignore ublox_gps ublox_msgs ublox_serialization
```

Or use `src/docker/build_ros2_workspace.sh` inside Docker.
