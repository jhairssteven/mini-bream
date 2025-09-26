Read a directory with rosbags and apply the topic filter defined in the .yaml file. For the output rosbag name use the same as the input rosbag, plust the uri defined in the .yaml, used as postfix.
```bash
for bag in rosbag2_*; 
do   
new_uri="${bag}_gps_only";
sed -i "s|^- uri:.*|- uri: ${new_uri}|" gpss_filter.yaml;
ros2 bag convert -i "$bag" -o gpss_filter.yaml; 
done
```