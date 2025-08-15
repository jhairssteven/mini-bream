#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, String

from frontseat.drivers.motors.BlueRoboticsT200 import BlueRoboticsT200

class MotorControllerNode(Node):
    def __init__(self, node_name='motor_controller', lft_motorDriver=None, right_motorDriver=None):
        
        if lft_motorDriver is None or right_motorDriver is None:
            raise ValueError("Missing required argument: A '__motorDriver' be defined")
        
        self.node_name = node_name
        self.left_motor_pwm_driver = lft_motorDriver
        self.right_motor_pwm_driver = right_motorDriver    
    
        super().__init__(self.node_name)
        
        # For critical real-time commands, reliable but no stored history
        reliable_volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Rety until success
            durability=QoSDurabilityPolicy.VOLATILE,    # Do not store old messages
            depth=5
        )
        
        # Subscribers
        self.right_sub = self.create_subscription(Float32, '/pwm/right_thrust_cmd', self.__right_thrust_cbk, reliable_volatile_qos)
        self.left_sub = self.create_subscription(Float32, '/pwm/left_thrust_cmd', self.__left_thrust_cbk, reliable_volatile_qos)

        # Publishers
        self.motor_techs_pub = self.create_publisher(String, '/pwm/motor_techs', 1)


    def __left_thrust_cbk(self, msg):
        self.left_motor_pwm_driver.setThrust(thrust=msg.data)
        #self.left_motor_techs = self.left_motor_pwm_controller.__techs()
    
    def __right_thrust_cbk(self, msg):
        self.right_motor_pwm_driver.setThrust(thrust=msg.data)
        #self.right_motor_techs = self.left_motor_pwm_controller.__techs()


def main(args=None):
    rclpy.init(args=args)

    left_mtr_GPIO_pin, right_mtr_GPIO_pin = [32, 33]  # Jetson Orin Mini-Bream

    left_motorDriver = BlueRoboticsT200(pin=left_mtr_GPIO_pin, motor_name="left_mtr")
    right_motorDriver = BlueRoboticsT200(pin=right_mtr_GPIO_pin, motor_name="right_mtr")

    motor_controller_node = MotorControllerNode(
        node_name='motor_controller',
        lft_motorDriver=left_motorDriver,
        right_motorDriver=right_motorDriver
    )

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
