import codecs
import os
import pathlib
import re
import subprocess

import sdformat13 as sdf
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from blueboat_sim.vrx import bridges as vrx_bridges
from blueboat_sim.vrx import payload_bridges as vrx_payload_bridges

USVS = ['usv', 'wam-v', 'blueboat']

_PACKAGE_URI_RE = re.compile(r'package://([^/]+)/([^\s"\'<>]+)')


def _resolve_package_uris(urdf_str: str) -> str:
    """Convert package:// URIs to absolute paths for gz sdf / Gazebo mesh loading."""

    def _replace(match: re.Match) -> str:
        pkg_name = match.group(1)
        rel_path = match.group(2)
        abs_path = os.path.join(get_package_share_directory(pkg_name), rel_path)
        if not os.path.exists(abs_path):
            raise RuntimeError(f'Mesh or resource not found: {abs_path}')
        return abs_path

    return _PACKAGE_URI_RE.sub(_replace, urdf_str)


def _spawn_tmp_dir():
    return os.path.join(
        get_package_share_directory('blueboat_sim'),
        'models',
        'spawn_tmp',
    )


class Model:

    def __init__(self, model_name, model_type, position):
        self.model_name = model_name
        self.model_type = model_type
        self.position = position
        self.payload = {}
        self.urdf = ''

    def is_USV(self):
        return self.model_type in USVS

    def bridges(self, world_name):
        return [[
            vrx_bridges.pose(self.model_name),
            vrx_bridges.pose_static(self.model_name),
            vrx_bridges.joint_states(world_name, self.model_name),
        ], [], []]

    def payload_bridges(self, world_name, payloads=None):
        if not payloads:
            payloads = self.payload
        return self._payload_bridges_impl(world_name, payloads)

    def _payload_bridges_impl(self, world_name, payloads):
        bridges = []
        nodes = []
        for sensor_name, value in payloads.items():
            link_name = value[0]
            sensor_type = value[1]
            bridges.extend(
                vrx_payload_bridges.payload_bridges(
                    world_name, self.model_name, link_name, sensor_name, sensor_type))

            if sensor_type == sdf.Sensortype.CAMERA:
                ros_sensor_prefix = f'sensors/cameras/{sensor_name}'
                nodes.append(Node(
                    package='vrx_ros',
                    executable='optical_frame_publisher',
                    arguments=['1'],
                    remappings=[
                        ('input/image', f'{ros_sensor_prefix}/image_raw'),
                        ('output/image', f'{ros_sensor_prefix}/optical/image_raw'),
                        ('input/camera_info', f'{ros_sensor_prefix}/camera_info'),
                        ('output/camera_info', f'{ros_sensor_prefix}/optical/camera_info'),
                    ]))

        return [bridges, nodes, []]

    def xacro_cmd(self):
        xacro_command = ['xacro', self.urdf, f'namespace:={self.model_name}',
                         'locked:=true', 'vrx_sensors_enabled:=true', 'thruster_config:=H']
        xacro_process = subprocess.Popen(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = xacro_process.communicate()
        urdf_str = codecs.getdecoder('unicode_escape')(stdout)[0]
        if xacro_process.returncode != 0:
            err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
            raise RuntimeError(f'xacro failed: {err_output}')

        urdf_str = _resolve_package_uris(urdf_str)

        model_tmp_dir = _spawn_tmp_dir()
        os.makedirs(model_tmp_dir, exist_ok=True)
        model_output_file = os.path.join(model_tmp_dir, 'model.urdf')
        with open(model_output_file, 'w') as f:
            f.write(urdf_str)
        return ['gz', 'sdf', '-p', model_output_file]

    def generate(self):
        if not self.urdf:
            self.urdf = os.path.join(
                get_package_share_directory('blueboat_sim'),
                'urdf',
                'blueboat_sim.urdf.xacro',
            )
        command = self.xacro_cmd()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        if process.returncode != 0:
            err_output = codecs.getdecoder('unicode_escape')(stderr)[0]
            raise RuntimeError(f'gz sdf failed: {err_output}')
        model_sdf = codecs.getdecoder('unicode_escape')(stdout)[0]
        self.payload = self.payload_from_sdf(model_sdf)
        return command, model_sdf

    def name_from_plugin(self, plugin_sdf):
        result = re.search(r'/*<name>(.*)<\/name>', plugin_sdf)
        if result:
            return result.group(1)

    def payload_from_sdf(self, model_sdf):
        payload = {}
        root = sdf.Root()
        root.load_sdf_string(model_sdf)
        model = root.model()
        for link_index in range(model.link_count()):
            link = model.link_by_index(link_index)
            for sensor_index in range(link.sensor_count()):
                sensor = link.sensor_by_index(sensor_index)
                payload[sensor.name()] = [link.name(), sensor.type()]
        for plugin in model.plugins():
            if plugin.name() == 'gz::sim::systems::Thruster':
                name = self.name_from_plugin(plugin.__str__())
                payload['thruster_thrust_' + name] = [link.name(), name]
            elif plugin.name() == 'gz::sim::systems::JointPositionController':
                name = self.name_from_plugin(plugin.__str__())
                payload['thruster_rotate_' + name] = [link.name(), name]
            else:
                payload[plugin.name()] = ['', plugin.filename()]
        return payload

    def write_spawn_sdf(self, model_sdf=None):
        if not model_sdf:
            _, model_sdf = self.generate()
        model_tmp_dir = _spawn_tmp_dir()
        os.makedirs(model_tmp_dir, exist_ok=True)
        model_sdf_file = os.path.join(model_tmp_dir, 'spawn.sdf')
        with open(model_sdf_file, 'w') as f:
            f.write(model_sdf)
        return model_sdf_file

    def spawn_service_request(self, world_name, model_sdf=None):
        sdf_file = self.write_spawn_sdf(model_sdf)
        return (
            f'sdf_filename: "{sdf_file}", '
            f'name: "{self.model_name}", '
            f'pose: {{position: {{x: {self.position[0]}, y: {self.position[1]}, '
            f'z: {self.position[2]}}}, orientation: {{w: 1}}}}'
        )

    def set_urdf(self, urdf):
        self.urdf = urdf

    @classmethod
    def FromConfig(cls, stream):
        config = yaml.safe_load(stream)
        if isinstance(config, list):
            return [cls._FromConfigDict(entry) for entry in config]
        if isinstance(config, dict):
            return cls._FromConfigDict(config)
        raise RuntimeError('Invalid model config')

    @classmethod
    def _FromConfigDict(cls, config):
        if 'model_name' not in config:
            raise RuntimeError('Cannot construct model without model_name in config')
        if 'model_type' not in config:
            raise RuntimeError('Cannot construct model without model_type in config')

        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if 'position' in config:
            if 'xyz' in config['position']:
                xyz = config['position']['xyz']
            if 'rpy' in config['position']:
                rpy = config['position']['rpy']
        model = cls(config['model_name'], config['model_type'], [*xyz, *rpy])
        return model
