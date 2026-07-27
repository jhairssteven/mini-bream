from blueboat_sim.vrx.bridge import Bridge, BridgeDirection


def pose(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)


def pose_static(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)


def joint_states(world_name, model_name):
    return Bridge(
        gz_topic=f'/world/{world_name}/model/{model_name}/joint_state',
        ros_topic='joint_states',
        gz_type='gz.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.GZ_TO_ROS)


def clock():
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='gz.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)


def usv_wind_speed():
    return Bridge(
        gz_topic='/vrx/debug/wind/speed',
        ros_topic='/vrx/debug/wind/speed',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)


def usv_wind_direction():
    return Bridge(
        gz_topic='/vrx/debug/wind/direction',
        ros_topic='/vrx/debug/wind/direction',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)


def task_info():
    return Bridge(
        gz_topic='/vrx/task/info',
        ros_topic='/vrx/task/info',
        gz_type='gz.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)
