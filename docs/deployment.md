# Deployment
Connect to the RPi
```shell
mkdir mini_bream_ws/src
# Run the sync_ws.sh on your computer
git clone https://github.com/KumarRobotics/ublox.git/
```
- build workspace
```shell
#cd to workspace root
catkin_make 
source devel/setup.bash
```

### Running a mission
To run a mission follows same steps as on [here](../src/autonomous_catamaran_ws/src/README.md#running-a-sample-mission).

**Do not forget to update the initial GPS position (goal_lat, goal_lon) on the `params/*.yaml` file of your vehicle.**

- Launch the ImuNode.py. Because Imu needs calibration frequently, it is better to launch it separately instead of with the launch files.
```shell
cd src/frontseat/src
python3 ImuNode.py
```