## Getting started
This will show you how to run a sample mission using the VRX simulator. For steps on how to deploy on real vehicle look at [deployment](../../../docs/deployment.md) documentation.

### Running a sample mission
> _**You will need four terminals opened in the root of your ROS workspace**_

- It is recomended to have a roscore run in a separate terminal, instead of letting roslaunch initiating it. Because some nodes normally need frequent rebooting, having a separate roscore prevents having to also reboot the visualizers such as Rviz or Foxglove.
```shell
#cd to workspace root
source devel/setup.bash
roscore
```

- This launched all nodes to give sensor data. Use `sim:=true` to get from simulation, otherwise to get directly from sensors drivers.
```shell
# if running on the actual vehicle put sim:=false
roslaunch frontseat all.launch sim:=true
```

> _**For the commands below that use param `vehicle`, if simulating, replace `<file_name>` with `vrx` (default). Otherwise, use one of the availables in `backseat/src/params` directory.**_
> _**If you run into "cannot launch node of type ..." error then execute below command and try again.***_
```shell
cd /workspace
chmod -R u+x /workspace
# this is a temporary fix, actual solution may something to do with USERIDs or group membership mismatch between the container and the localhost.
```

- Launches tools to visualize current pose and trajectory on Rviz.
```shell
roslaunch visualization_tools visuals.launch vehicle:=<file_name>
```
- Launch path planner.
```shell
roslaunch backseat mission.launch vehicle:=<file_name>
```

- Launch the mission planner with custom mission. Replace `<mission_file_name>` with one from the `mission_planner/missions` directory.
```shell
roslaunch mission_planner action_client.launch client_type:=FILE name:=<mission_file_name> vehicle:=<file_name>
```
> **Note: If `Mission_complete=False` appears on the log just (or the mission trajectory doesn't show up on RViz) try running again the launch file. This behaviour is a known issue (see [issues](../../../README.md#issues)).**

- If you want to create a new mission, follow the directory structure and file names of the sample missions. No code changes need to be made. See [usage](mission_planner/README.md)
