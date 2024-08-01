## Ground station
Ground station refers to the computer where you can view the realtime status of the boat while on field tests. This guide shows how to do the setup.

### Requirements
- Both the RPi and the computer should be connected to the same network.
- At least the all.launch frontseat package should be running on the vehicle.

### Setup
- Log into the docker container as described [here](../README.md#build-the-docker-image-and-run-the-container).

- Update the ROS_MASTER_URI to point to the RPi

```shell
nano ~/.bashrc
# set ROS_MASTER_URI=http://raspberrypi:11311
source ~/.bashrc
sudo nano /etc/hosts
# Add the line
# 192.168.168.100 raspberrypi
# Ensure you can get data by echoing gps topic
rostopic echo /ublox_gps/fix
```

- On the RPi
```shell
sudo nano /etc/hosts
# <ip_of_your_computer> <host_name_of_your_computer>
```
- Run the visuals.launch file and ensure you get data on the RPi from your computer with
```shell
rostopic echo /curr_marker
```

## Remote teleoperation
You can control the vehicle through an Xbox controller.
```shell
# Connect to the controller through Bluetooth
rosrun joy joy_node
# When moving the joysticks you should get data when echoing topic /joy
```
- The three-line button on the middle of the controller, determines wheter motor control is given by the Xbox controller or by the pathPlanner. An activation message should appear at the terminal running the frontseat/all.launch node. It tells _Radio on control: True/False_.