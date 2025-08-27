#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from frontseat.qos_profiles import best_effort_volatile_qos

class JoyController(Node):
    def __init__(self, node_name='joystick'):
        super().__init__(node_name)
        self.node_name = node_name
        self.left_mtr_btn_axis = 1  # left joystick vertical axis
        self.right_mtr_btn_axis = 4 # 3 for bluetooth #4 for wired connection # right joystick vertical axis
        self.radio_on_ctrl_btn = 11
        self.stop_motors_btn = 10
        self.radio_on_ctrl = True

        # Subscriber
        self.subscriber = self.create_subscription(
            Joy,
            "/joy",
            self.__joy_cbk,
            best_effort_volatile_qos
        )

        # Publishers
        self.left_thrust_pub = self.create_publisher(Float32, '/radio/thrusters/left_cmd', best_effort_volatile_qos)
        self.right_thrust_pub = self.create_publisher(Float32, '/radio/thrusters/right_cmd', best_effort_volatile_qos)
        self.radio_on_ctrl_pub = self.create_publisher(Bool, '/radio/on_ctrl', best_effort_volatile_qos)

        self.__log_radio_on_ctrl()

    def __log_radio_on_ctrl(self):
        self.get_logger().warn(f'Radio on control: {self.radio_on_ctrl}')

    def __joy_cbk(self, data: Joy):
        if len(data.axes) >= 7:
            left_thrust = data.axes[self.left_mtr_btn_axis]
            right_thrust = data.axes[self.right_mtr_btn_axis]

            self.left_thrust_pub.publish(Float32(data=left_thrust))
            self.right_thrust_pub.publish(Float32(data=right_thrust))

        if (data.buttons[self.radio_on_ctrl_btn]):  # Take-over button was pressed then toggle radio_on_ctrl
            self.radio_on_ctrl = not self.radio_on_ctrl
            self.__log_radio_on_ctrl()
            self.radio_on_ctrl_pub.publish(Bool(data=self.radio_on_ctrl))

        if (data.buttons[self.stop_motors_btn]):  # Stop the motors button was pressed
            self.left_thrust_pub.publish(Float32(data=0.0))
            self.right_thrust_pub.publish(Float32(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    joy_to_motors = JoyController()
    rclpy.spin(joy_to_motors)
    joy_to_motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
