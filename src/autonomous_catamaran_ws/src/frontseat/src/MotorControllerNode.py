#! /usr/bin/python3
import rospy
from std_msgs.msg import Float32, String

class MotorControllerNode:
    def __init__(self, node_name='motor_controller', lft_motorDriver=None, right_motorDriver=None):
        if lft_motorDriver is None or right_motorDriver is None:
            raise ValueError("Missing required argument: A '__motorDriver' be defined")
        self.node_name = node_name
        self.left_motor_pwm_controller = lft_motorDriver
        self.right_motor_pwm_controller = right_motorDriver
        self.__init_node()
    
    def __init_node(self):
        rospy.init_node(self.node_name)
        rospy.Subscriber('/pwm/right_thrust_cmd', Float32, self.__right_thrust_cbk, queue_size=10)
        rospy.Subscriber('/pwm/left_thrust_cmd', Float32, self.__left_thrust_cbk, queue_size=10)

        # Debug
        self.motor_techs_pub = rospy.Publisher('/pwm/motor_techs', String, queue_size=1)

    def __left_thrust_cbk(self, msg):
        self.left_motor_pwm_controller.setThrust(thrust=msg.data)
        #self.left_motor_techs = self.left_motor_pwm_controller.__techs()
    
    def __right_thrust_cbk(self, msg):
        self.right_motor_pwm_controller.setThrust(thrust=msg.data)
        #self.right_motor_techs = self.left_motor_pwm_controller.__techs()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    # [13, 12] # BREAM.
    left_mtr_GPIO_pin, rigth_mtr_GPIO_pin = [18, 13] # mini-bream
    
    from drivers.BlueRoboticsT200 import PWMMotorController as T200_MC
    from drivers.BREAM_MC import PWMMotorController as BREAM_MC
    
    right_motorDriver = BREAM_MC(pin=left_mtr_GPIO_pin, motor_name="right_mtr")
    lft_motorDriver = BREAM_MC(pin=rigth_mtr_GPIO_pin, motor_name="left_mtr")
    motor_controller_node = MotorControllerNode(node_name='motor_controller', lft_motorDriver=lft_motorDriver, right_motorDriver=right_motorDriver)
    motor_controller_node.run()