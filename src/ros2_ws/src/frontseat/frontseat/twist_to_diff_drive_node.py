#!/usr/bin/env python3
"""
Twist to Differential Drive Node

Subscribes to /cmd_vel (geometry_msgs/Twist) and converts to
differential drive commands for WAM-V left and right thrusters.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TwistToDiffDriveNode(Node):
    """
    Converts Twist commands to differential drive thruster commands.
    
    Subscribes to:
        /cmd_vel (geometry_msgs/Twist): Velocity commands
    
    Publishes to:
        /wamv/thrusters/left/thrust (std_msgs/Float64): Left thruster command
        /wamv/thrusters/right/thrust (std_msgs/Float64): Right thruster command
    """
    
    def __init__(self):
        super().__init__('twist_to_diff_drive_node')
        
        # Parameters for differential drive conversion
        self.declare_parameter('wheel_separation', 1.0)  # Distance between thrusters (meters)
        self.declare_parameter('max_velocity', 1.0)  # Maximum expected velocity in m/s
        self.declare_parameter('max_thrust', 1.0)  # Maximum thrust percentage
        self.declare_parameter('min_thrust', -1.0)  # Minimum thrust percentage
        self.declare_parameter('thrust_range', 1000.0)  # WAM-V thruster range
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.thrust_range = self.get_parameter('thrust_range').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.min_thrust = self.get_parameter('min_thrust').value
        
        # QoS Profile
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        qos_reliable_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Retry until success
            durability=QoSDurabilityPolicy.VOLATILE,    # Do not store old messages
            depth=5
        )
        
        # Subscriber to cmd_vel
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_best_effort
        )
        
        # Publishers for thrusters
        self.left_thrust_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            qos_reliable_volatile
        )
        
        self.right_thrust_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            qos_reliable_volatile
        )
        
        self.get_logger().info('Twist to Differential Drive Node initialized')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'Max velocity: {self.max_velocity}m/s')
        self.get_logger().info(f'Thrust range: [{self.min_thrust*100.0}%, {self.max_thrust*100.0}%]')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist message to differential drive commands.
        
        Differential drive kinematics:
        - linear.x: forward/backward velocity
        - angular.z: rotational velocity
        
        Left and right wheel velocities:
        - v_left = linear.x - (angular.z * wheel_separation / 2)
        - v_right = linear.x + (angular.z * wheel_separation / 2)
        """
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate differential drive velocities (in m/s)
        v_left = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Convert from m/s to percentage (-100 to 100)
        # Percentage = (velocity / max_velocity) * 100
        v_left_percent = (v_left / self.max_velocity)
        v_right_percent = (v_right / self.max_velocity)
        
        # Clamp to thrust limits
        v_left_percent = max(self.min_thrust, min(self.max_thrust, v_left_percent))
        v_right_percent = max(self.min_thrust, min(self.max_thrust, v_right_percent))
        
        # Publish thrust commands
        left_msg = Float64()
        left_msg.data = v_left_percent * self.thrust_range
        self.left_thrust_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = v_right_percent * self.thrust_range
        self.right_thrust_pub.publish(right_msg)
        
        self.get_logger().info(
            f'cmd_vel: linear={linear_vel:.3f}m/s, angular={angular_vel:.3f}rad/s -> '
            f'thrusters: left={v_left_percent:.1f}%, right={v_right_percent:.1f}%'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistToDiffDriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
