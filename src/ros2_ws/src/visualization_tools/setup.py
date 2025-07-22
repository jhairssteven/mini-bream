from setuptools import find_packages, setup
from glob import glob

package_name = 'visualization_tools'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*')),
        ('share/' + package_name + '/visualizers_config/rviz_config/', glob('visualizers_config/rviz_config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Gallego',
    maintainer_email='root@todo.todo',
    description='The visualization_tools package helps with rviz config files and python topic adapters',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_visuals_adapter = visualization_tools.RvizVisualsAdapter:main',
            'interactive_path = visualization_tools.interactive_path_node:main'
        ],
    },
)
