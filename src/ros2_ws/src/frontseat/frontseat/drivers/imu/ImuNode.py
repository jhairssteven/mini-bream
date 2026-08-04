#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from frontseat.drivers.imu.HMC6343 import HMC6343_handler
import argparse

class ImuNode(Node):
    def __init__(self, node_name='compass', _IMUDriver=None):
        super().__init__(node_name)

        if _IMUDriver is None:
            raise ValueError("Missing required argument '_IMUDriver': Must pass IMU sensor driver")

        self._IMUDriver = _IMUDriver
        self.compass_bearing_pub = self.create_publisher(Float32, '/compass_bearing_deg', 10)
        self.timer = self.create_timer(1.0 / 3.0, self.publish_bearing)  # 3 Hz
        self.get_logger().warn('This node has not been tested on Jetson platforms')

    def publish_bearing(self):
        try:
            compass_bearing_deg = self._IMUDriver.getHeading()
            msg = Float32()
            msg.data = compass_bearing_deg
            self.compass_bearing_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading IMU: {e}")

def main(args=None):
    parser = argparse.ArgumentParser(description='IMU Node with calibration option')
    parser.add_argument('--calibrate', type=bool, default=False, help='Start IMU calibration process? (True or False)')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    imu_node = ImuNode(_IMUDriver=HMC6343_handler(calibrate=parsed_args.calibrate))
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
