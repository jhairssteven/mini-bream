from setuptools import find_packages, setup
from glob import glob

package_name = 'frontseat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config' + '/cameras', glob('config/cameras/*')),
        ('share/' + package_name + '/config' + '/ublox_gps', glob('config/ublox_gps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='71862429+jhairssteven@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = frontseat.MotorControllerNode:main',
            'thrust_source_selector = frontseat.ThrustSourceSelectorNode:main',
            'joystick = frontseat.JoystickHandlerNode:main',
            'imu_estimation = frontseat.ImuFromGPS:main',
            'dual_antenna = frontseat.DualAntenna:main',
            'moving_base_rtk = frontseat.MovingBaseRTK:main',
            'gps_center_offset_node = frontseat.GpsCenterOffsetNode:main',
        ],
    },
)
