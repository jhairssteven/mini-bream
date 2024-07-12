sudo rosdep init
rosdep update
cd /workspace/autonomous_catamaran_ws
rosdep install --from-paths src --ignore-src -r -y
echo "------------------"
echo "------------------"
echo "--------INSTALLED DEPS-------"