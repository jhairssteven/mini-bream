#! /usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool

class JoyController:
    def __init__(self, node_name='joy_controller'):
        self.node_name = node_name
        self.left_mtr_btn_axis = 1  # left joystick vertical axis
        self.right_mtr_btn_axis = 3 # 3 for bluetooth #4 for wired connection # right joystick vertical axis
        self.radio_on_ctrl = True
        self.init_node()

    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.subscriber = rospy.Subscriber("/joy", Joy, self.__joy_cbk)

        #self.left_thrust_pub = rospy.Publisher('/radio_left_thrust_cmd', Float32, queue_size=1)
        self.left_thrust_pub = rospy.Publisher('/radio/thrusters/left_cmd', Float32, queue_size=1)
        self.right_thrust_pub = rospy.Publisher('/radio/thrusters/right_cmd', Float32, queue_size=1)
        self.radio_on_ctrl_pub = rospy.Publisher('/radio/on_ctrl', Bool, queue_size=1)
        
        self.__log_radio_on_ctrl()

    def __log_radio_on_ctrl(self):
        rospy.logwarn(f'Radio on control: {self.radio_on_ctrl}')

    def __joy_cbk(self, data):
        if len(data.axes) >= 7:
            left_thrust = data.axes[self.left_mtr_btn_axis]
            right_thrust = data.axes[self.right_mtr_btn_axis]
            
            self.left_thrust_pub.publish(left_thrust)
            self.right_thrust_pub.publish(right_thrust)
        
        if (data.buttons[11] == 1): # Take-over button was pressed then toggle radio_on_ctrl
            self.radio_on_ctrl = not self.radio_on_ctrl
            self.__log_radio_on_ctrl()
            self.radio_on_ctrl_pub.publish(self.radio_on_ctrl)

        if (data.buttons[10] == 1): # Stop button was pressed
            self.left_thrust_pub.publish(0)
            self.right_thrust_pub.publish(0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    joy_to_motors = JoyController()
    joy_to_motors.run()