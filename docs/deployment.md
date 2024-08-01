# Deployment
Connect to the RPi through SSH (see the [Connecting to the RPi](port_to_new_rpi.md#connecting-to-the-rpi) for reference).
```shell
mkdir mini_bream_ws/src
# Run the sync_ws.sh from the ground station computer to syncronize any source code changes.
```
- build workspace
```shell
#cd to workspace root
catkin_make 
source devel/setup.bash
```

### Running a mission
- Ensure you have a 'roscore' running before
- Launch the ImuNode.py. Because Imu needs calibration frequently, it is better to launch it separately instead of with the launch files.
```shell
cd src/frontseat/src
python3 ImuNode.py # run with --calibrate=True if you need to calibrate the IMU
```

> To calibrate the IMU, after launch the calibration, rotate once in the x axes and then in the z axes, put the IMU back to the velcro and then continue with the instructions that outputs the ImuNode.py script.

To run a mission follows same steps as on [here](../src/autonomous_catamaran_ws/src/README.md#running-a-sample-mission).

**Do not forget to update the initial GPS position (goal_lat, goal_lon) on the `params/*.yaml` file of your vehicle.**

### Monitoring
If you would like to see real-time vehicle trajectory and target mission, please refer to the [ground station setup](ground_station_setup.md) guide.