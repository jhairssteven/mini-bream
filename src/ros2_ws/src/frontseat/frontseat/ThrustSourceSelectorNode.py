#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from frontseat.qos_profiles import reliable_volatile_qos, best_effort_volatile_qos

class ThrustSourceSelectorNode(Node):
    """ 
    This node works as a demultiplexer by subscribing to radio and
    path planner thrust commands. It outputs or routes only one of these
    topics based on whether the radio has thrust control.
    """

    """ 
    This node works as a demultiplexer by subscribing to two topics 
    that command motor thrust values. It outputs or routes only one of these
    topics based on the value of a specified variable.
    """

    def __init__(self, node_name="thrust_source_selector"):
        super().__init__(node_name)
        self.radio_has_control = True
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        
        # Subscribers
        self.create_subscription(Float32, '/wamv/thrusters/left_thrust_cmd', self.__frontseat_left_thrust_cbk, best_effort_volatile_qos)
        self.create_subscription(Float32, '/wamv/thrusters/right_thrust_cmd', self.__frontseat_right_thrust_cbk, best_effort_volatile_qos)
        self.create_subscription(Float32, '/radio/thrusters/left_cmd', self.__radio_left_thrust_cbk, best_effort_volatile_qos)
        self.create_subscription(Float32, '/radio/thrusters/right_cmd', self.__radio_right_thrust_cbk, best_effort_volatile_qos)
        self.create_subscription(Bool, '/radio/on_ctrl', self.__radio_on_ctrl_cbk, best_effort_volatile_qos)

        # Publishers
        self.pwm_right_thrust_pub = self.create_publisher(Float32, '/pwm/right_thrust_cmd', reliable_volatile_qos)
        self.pwm_left_thrust_pub = self.create_publisher(Float32, '/pwm/left_thrust_cmd', reliable_volatile_qos)

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.__publish_thrust)

    def __radio_on_ctrl_cbk(self, msg: Bool):
        self.radio_has_control = msg.data

    def __radio_right_thrust_cbk(self, msg: Float32):
        if self.radio_has_control:
            self.right_thrust = msg.data

    def __radio_left_thrust_cbk(self, msg: Float32):
        if self.radio_has_control:
            self.left_thrust = msg.data

    def __frontseat_right_thrust_cbk(self, msg: Float32):
        if not self.radio_has_control:
            self.right_thrust = msg.data

    def __frontseat_left_thrust_cbk(self, msg: Float32):
        if not self.radio_has_control:
            self.left_thrust = msg.data

    def __publish_thrust(self):
        self.pwm_right_thrust_pub.publish(Float32(data=self.right_thrust))
        self.pwm_left_thrust_pub.publish(Float32(data=self.left_thrust))


def main(args=None):
    rclpy.init(args=args)
    node = ThrustSourceSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
