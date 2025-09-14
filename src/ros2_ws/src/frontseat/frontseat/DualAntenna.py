#!/usr/bin/env python3
from dataclasses import dataclass
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import utm
from frontseat.qos_profiles import reliable_transient_local_qos, best_effort_volatile_qos, reliable_volatile_qos
import numpy as np
from std_msgs.msg import String
import rclpy

@dataclass
class GPS:
    lat: float
    lon: float

    def to_utm(self):
        utm_x, utm_y, _, _ = utm.from_latlon(self.lat, self.lon)
        return utm_x, utm_y
    
class DualAntenna(Node):
    """ Heading estimation using a dual antenna (GPS) """
    def __init__(self, node_name='dual_antenna'):
        super().__init__(node_name)
        #self.origin_gps = GPS(40.448417, -86.867750) # Harner
        self.origin_gps = GPS(40.40229852, -86.84558228) # Kepner
        self.ox, self.oy = self.origin_gps.to_utm()
        self.topic_gps1 = '/fix1'
        self.topic_gps2 = '/fix2'
        self.create_subscription(NavSatFix, self.topic_gps1, self.gps1_cbk, reliable_volatile_qos)
        self.create_subscription(NavSatFix, self.topic_gps2, self.gps2_cbk, reliable_volatile_qos)
        self.stats_pub = self.create_publisher(String, '/dualAntenna/stats', reliable_volatile_qos)

        self.gps1 = self.origin_gps
        self.gps2 = self.origin_gps

        self.timer = self.create_timer(1.0 / 19.0, self.estimate_heading)
        
    def gps1_cbk(self, msg):
        self.gps1.lat, self.gps1.lon = msg.latitude, msg.longitude

    def gps2_cbk(self, msg):
        self.gps2.lat, self.gps2.lon = msg.latitude, msg.longitude

    def estimate_heading(self):
        gps1_x, gps1_y = self.gps1.to_utm()
        gps2_x, gps2_y = self.gps2.to_utm()
        Px = gps2_x-gps1_x
        Py = gps2_y-gps1_y
        angle = np.arctan2(Py, Px)
        separation = np.hypot(Px, Py)

        
        stats = f'{gps1_x-self.ox}, {gps1_y-self.oy}, {gps2_x-self.ox}, {gps2_y-self.oy}, {Px}, {Py}, {angle}, {separation}'
        self.stats_pub.publish(String(data=stats))

def main(args=None):
    rclpy.init()

    dualAntenna = DualAntenna()
    try:
        rclpy.spin(dualAntenna)
    except KeyboardInterrupt:
        pass
    finally:
        dualAntenna.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
