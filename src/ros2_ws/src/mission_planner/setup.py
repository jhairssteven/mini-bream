from setuptools import find_packages, setup
from glob import glob
package_name = 'mission_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Gallego',
    maintainer_email='root@todo.todo',
    description='The mission_planner package. This stores missions and has scripts to process gps and xy based missions from csv files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_goal_to_waypoint = mission_planner.nav_goal_to_waypoint:main'
        ],
    },
)
