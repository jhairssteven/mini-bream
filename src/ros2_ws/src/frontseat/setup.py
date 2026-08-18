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
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config' + '/cameras', glob('config/cameras/*')),
        ('share/' + package_name + '/config' + '/ublox_gps', glob('config/ublox_gps/*')),
        ('share/' + package_name + '/config' + '/rslidar_airy', glob('config/rslidar_airy/*')),
        ('share/' + package_name + '/config' + '/zed2i', glob('config/zed2i/*.yaml') + glob('config/zed2i/*.md')),
        ('share/' + package_name + '/config' + '/zed2i/settings', glob('config/zed2i/settings/*')),
        ('share/' + package_name + '/config' + '/heading', glob('config/heading/*')),
        ('share/' + package_name + '/config' + '/tf', glob('config/tf/*')),
        ('share/' + package_name + '/config' + '/self_filter', glob('config/self_filter/*')),
        ('share/' + package_name + '/config' + '/ransac', glob('config/ransac/*')),
        ('share/' + package_name + '/config' + '/pc_clustering', glob('config/pc_clustering/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes/blueboat', glob('meshes/blueboat/*')),
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
            'rel_pos_heading = frontseat.RelPosHeadingNode:main',
            'heading_ekf = frontseat.HeadingEkfNode:main',
            'gps_map_odom = frontseat.GpsMapOdomNode:main',
            'odom_tf_broadcaster = frontseat.OdomTfBroadcasterNode:main',
            'static_tf_broadcaster = frontseat.StaticTfBroadcasterNode:main',
            'self_filter = frontseat.SelfFilterNode:main',
            'waterline_ransac = frontseat.ransac.WaterlineRansacNode:main',
            'pc_clustering = frontseat.pc_clustering.PcClusteringNode:main',
        ],
    },
)
