from blueboat_sim.vrx.bridge import Bridge, BridgeDirection

import sdformat13 as sdf


def gz_prefix(world_name, model_name, link_name, sensor_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}'


def ros_prefix(sensor_name, sensor_type):
    return f'sensors/{sensor_type}/{sensor_name}'


def image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/image',
        ros_topic=f'{ros_sensor_prefix}/image_raw',
        gz_type='gz.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


def camera_info(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/camera_info',
        ros_topic=f'{ros_sensor_prefix}/camera_info',
        gz_type='gz.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.GZ_TO_ROS)


def lidar_scan(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan',
        ros_topic=f'{ros_sensor_prefix}/scan',
        gz_type='gz.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.GZ_TO_ROS)


def lidar_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='gz.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


def imu(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'imu')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/imu',
        ros_topic=f'{ros_sensor_prefix}imu/data',
        gz_type='gz.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS)


def navsat(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'gps')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/navsat',
        ros_topic=f'{ros_sensor_prefix}gps/fix',
        gz_type='gz.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS)


def odometry(model_name):
    ros_sensor_prefix = ros_prefix('', 'position')
    return Bridge(
        gz_topic=f'/model/{model_name}/odometry',
        ros_topic=f'{ros_sensor_prefix}ground_truth_odometry',
        gz_type='gz.msgs.Odometry',
        ros_type='nav_msgs/msg/Odometry',
        direction=BridgeDirection.GZ_TO_ROS)


def thrust(model_name, side):
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/thrust',
        ros_topic=f'thrusters/{side}/thrust',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def thrust_joint_pos(model_name, side):
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/pos',
        ros_topic=f'thrusters/{side}/pos',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)


def payload_bridges(world_name, model_name, link_name, sensor_name, sensor_type):
    if sensor_type == sdf.Sensortype.CAMERA:
        return [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name),
        ]
    if sensor_type == sdf.Sensortype.IMU:
        return [imu(world_name, model_name, link_name, sensor_name)]
    if sensor_type == sdf.Sensortype.NAVSAT:
        return [navsat(world_name, model_name, link_name, sensor_name)]
    if sensor_type == sdf.Sensortype.GPU_LIDAR:
        return [
            lidar_scan(world_name, model_name, link_name, sensor_name),
            lidar_points(world_name, model_name, link_name, sensor_name),
        ]
    if 'OdometryPublisher' in sensor_name:
        return [odometry(model_name)]
    if 'thruster_thrust_' in sensor_name:
        return [thrust(model_name, sensor_type)]
    if 'thruster_rotate_' in sensor_name:
        return [thrust_joint_pos(model_name, sensor_type)]
    return []
