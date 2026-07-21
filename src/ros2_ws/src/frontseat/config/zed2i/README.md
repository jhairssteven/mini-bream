# ZED 2i (Jetson perception container)

Runs in the Jetson `perception` service (`mini-bream:perception` image). Publishes only:

- `/zed/zed/rgb/color/rect/image` (`sensor_msgs/Image`)
- `/zed/zed/rgb/color/rect/camera_info` (`sensor_msgs/CameraInfo`)
- `/zed/zed/point_cloud/cloud_registered` (`sensor_msgs/PointCloud2`)

## Offline calibration file

The ZED SDK downloads `SN<serial>.conf` on first open. For offline use, the file is
checked into `settings/` and mounted into the container at `/usr/local/zed/settings`.

Current camera: serial **37419761** → `settings/SN37419761.conf`

If you swap cameras, download the new file once (with internet):

1. Connect the camera and run the Jetson container once online, **or**
2. Download from [calib.stereolabs.com](https://calib.stereolabs.com) and place
   `SN<serial>.conf` in `settings/`.

`resources/` is mounted for optional SDK assets; PERFORMANCE depth mode avoids
neural model downloads.

## Launch

Started automatically by `src/docker/start_perception.sh` in tmux session `zed2i`.

Manual:

```bash
ros2 launch frontseat zed2i_camera.launch.py
```

## Verify

```bash
ros2 topic list | grep zed
ros2 topic hz /zed/zed/rgb/color/rect/image
ros2 topic hz /zed/zed/point_cloud/cloud_registered
```
