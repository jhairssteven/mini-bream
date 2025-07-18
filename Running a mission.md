## Getting started
This will show you how to run a sample mission using the VRX simulator.

### Running a sample mission
> _**You will need four terminals opened in the root of your ROS workspace**_

> _**To load platform specific parameters, set the `vehicle` parameter to any filename in `backseat/backseat/params/`. The default is `vrx` (used for simulation).**_


> _If you run into "cannot launch node of type ..." error then execute below command and try again._
```shell
cd /workspace
chmod -R u+x /workspace
# this is a temporary fix, actual solution may something to do with USERIDs or group membership mismatch between the container and the localhost.
```

- Launch vrx simulator (when simulating only)
```shell
ros2 launch linc_gz docking_harner.launch.py
```

- Launches tools to visualize current pose and trajectories on Rviz.
```shell
ros2 launch visualization_tools visuals.launch.py vehicle:=<file_name>
```
- Launch path planner.
```shell
ros2 launch backseat mission.launch.py vehicle:=<file_name>
```

- Launch the mission planner with a custom mission. Replace `<mission_file_name>` with one from the `mission_planner/missions` directory.

Defaults:

```yaml
load_mission_from_file='false'
mission_name='mission'
vehicle='vrx
```

```shell
ros2 launch mission_planner action_client.launch.py load_mission_from_file:='true' mission_name:='<mission_file_name>' vehicle:='<file_name>'
```

> **Note: If `Mission_complete=False` appears on the log just (or the mission trajectory doesn't show up on RViz) try running again the launch file. This behaviour is a known issue (see [issues](../../../README.md#issues)).**

- If you want to create a new mission, follow the directory structure and file names of the sample missions. No code changes need to be made. See [usage](mission_planner/README.md)
