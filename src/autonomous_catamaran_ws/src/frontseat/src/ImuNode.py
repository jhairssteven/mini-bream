#! /usr/bin/python3
import rospy
from std_msgs.msg import Float32

import argparse

class ImuNode:
  def __init__(self, node_name='compass', _IMUDriver = None):
    self.node_name = node_name
    if _IMUDriver is None:
      raise ValueError("Missing required argument '_IMUDriver': Must pass IMU sensor driver")
    self._IMUDriver = _IMUDriver
    
    self.__init_node()

  def __init_node(self):
    rospy.init_node(self.node_name)
    self.compass_bearing_pub = rospy.Publisher('/compass_bearing_deg', Float32, queue_size=1)
    self.rate = rospy.Rate(3)


  def run(self):
    while not rospy.is_shutdown():
      try:
          compass_bearing_deg = self._IMUDriver.getHeading()
          self.compass_bearing_pub.publish(compass_bearing_deg)
          self.rate.sleep()
      except rospy.ROSInterruptException:
        break

def main(calibrate):
    from drivers.HMC6343 import HMC6343_handler

    imuNode = ImuNode(_IMUDriver=HMC6343_handler(calibrate=calibrate))
    imuNode.run()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='IMU Node with calibration option')
    parser.add_argument('--calibrate', type=bool, default=False, help='Set calibration mode (True or False)')
    args = parser.parse_args()

    main(args.calibrate)