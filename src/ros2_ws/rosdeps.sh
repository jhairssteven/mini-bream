sudo rosdep init
rosdep update
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
echo "------------------"
echo "------------------"
echo "--------INSTALLED DEPS-------"

sudo apt install ros-humble-tf-transformations # First installed for the visualization_tools pkg
sudo apt install ros-humble-geographic-msgs # First installed for the mission_planner pkg
pip install git+https://github.com/rdesc/pydubins.git@58c4b68d9ddf972dfd32afea295c3195da648cf2 # dubins support for python 3.10 (original pip install dubins only supports up to python3.8)