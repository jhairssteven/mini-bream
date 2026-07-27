from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'blueboat_sim'


def _install_tree(source_dir: str, install_subdir: str):
    entries = []
    for root, _dirs, files in os.walk(source_dir):
        if not files:
            continue
        rel_root = os.path.relpath(root, source_dir)
        target = os.path.join('share', package_name, install_subdir, rel_root)
        if rel_root == '.':
            target = os.path.join('share', package_name, install_subdir)
        entries.append((target, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/spawn_tmp', []),
        *_install_tree('urdf', 'urdf'),
        *_install_tree('meshes', 'meshes'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steven',
    maintainer_email='71862429+jhairssteven@users.noreply.github.com',
    description='BlueBoat VRX Gazebo simulation with HAL topic bridges.',
    license='Apache-2.0',
)
