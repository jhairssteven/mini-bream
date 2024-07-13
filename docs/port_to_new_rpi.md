# Step by step on how to install mini-breams libraries on new RPi

### Tested with this software
    No LSB modules are available.
    Distributor ID:	Ubuntu
    Description:	Ubuntu 20.04.6 LTS
    Release:	20.04
    Codename:	focal
    pi@raspberrypi:~$ 

### Credentials
- Power-on the mini-bream computing box
- Wi-Fi
```shell
    - SSID: `mini_bream`
    - PassW: somethin
```
- For RPi
```shell
    ssh pi@192.168.168.100
    passW: 1
```
If this doesn't work then connect to `naslab` WiFi and ssh RPi with 
- For RPi
```shell
    ssh pi@192.168.1.124
    passW: 1
```
### Setup
#### Prerequisites
- Ensure python3 is installed
- Also install pip3
- Install ros noetic
- install tmux
```shell
sudo apt install python3-pip
```

- Get the code (frontseat, backseat, Missions) folders

### Dependencies
- gps
```shell
#sudo apt-get install ros-noetic-rtcm-msgs
cd /autonomous_catamaran_ws/src
git clone https://github.com/KumarRobotics/ublox.git/
cd ..
```
-IMU HMC
```shell
sudo apt-get install -y i2c-tools
pip install quick2wire-api
```
- Motor controller (pigpiod)
```shell
#pip install pigpio
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

- Install ros deps

```shell
sudo rosdep init
rosdep update
cd /workspace/autonomous_catamaran_ws
rosdep install --from-paths src --ignore-src -r -y

#as single command
sudo rosdep init; rosdep update; cd /workspace/autonomous_catamaran_ws; rosdep install --from-paths src --ignore-src -r -y
```
- build workspace
```shell
#cd to workspace root
catkin_make 
source devel/setup.bash
```