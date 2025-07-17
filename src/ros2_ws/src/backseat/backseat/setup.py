from setuptools import find_packages, setup
from glob import glob

package_name = 'backseat'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('backseat/params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Gallego',
    maintainer_email='root@todo.todo',
    description='Implementation of the Dubins-based path follower with ILOS controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PathPlannerNode = backseat.PathPlannerNode:main'
        ],
    },
)
