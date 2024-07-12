#! /usr/bin/python3
import rospy
from std_msgs.msg import Float32, Bool

class ThrustSourceSelectorNode:
    """ This node works as a demultiplexer by subscribing to radio and
    path planner thrust commands. It outputs or routes only one of these
    topics based on whether the radio has thurst control. """
    
    """ This node works as a demultiplexer by subscribing two topics 
    that command motor thrust values. It outputs or routes only one of these
    topics based on the value of a specified variable."""
    
    def __init__(self, node_name="thrust_source_selector"):
        self.node_name = node_name
        self.radio_has_control = True
        self.left_thrust = 0
        self.right_thrust = 0
        self.__init_node()

    def __init_node(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.Subscriber('/wamv/thrusters/left_thrust_cmd', Float32, self.__frontseat_left_thrust_cbk, queue_size=10)
        rospy.Subscriber('/wamv/thrusters/right_thrust_cmd', Float32, self.__frontseat_right_thrust_cbk, queue_size=10)
        rospy.Subscriber('/radio/thrusters/left_cmd', Float32, self.__radio_left_thrust_cbk, queue_size=10)
        rospy.Subscriber('/radio/thrusters/right_cmd', Float32, self.__radio_right_thrust_cbk, queue_size=10)
        rospy.Subscriber('/radio/on_ctrl', Bool, self.__radio_on_ctrl_cbk, queue_size=10)

        self.pwm_right_thrust_pub = rospy.Publisher('/pwm/right_thrust_cmd', Float32, queue_size=10)
        self.pwm_left_thrust_pub = rospy.Publisher('/pwm/left_thrust_cmd', Float32, queue_size=10)
        
    def __radio_on_ctrl_cbk(self, msg):
        self.radio_has_control = msg.data
        
    def __radio_right_thrust_cbk(self, msg):
        if self.radio_has_control:
            self.right_thrust = msg.data

    def __radio_left_thrust_cbk(self, msg):
        if self.radio_has_control:
            self.left_thrust = msg.data

    def __frontseat_right_thrust_cbk(self, msg):
        if not self.radio_has_control:
            self.right_thrust = msg.data

    def __frontseat_left_thrust_cbk(self, msg):
        if not self.radio_has_control:
            self.left_thrust = msg.data
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pwm_right_thrust_pub.publish(self.right_thrust)
            self.pwm_left_thrust_pub.publish(self.left_thrust)
            #self.__sendToPWM(self.left_thrust, self.right_thrust)
            rate.sleep()

    #def __sendToPWM(self, left_thrust, right_thrust):
    #    """ Sends to the PWM controller the given left and right thrusts. 
    #    This is an interface to the actual PWM hardware implementation. """
    #    rospy.logerr(f'[TODO]: sendToPWM left: {left_thrust}, right: {right_thrust}')

if __name__ == "__main__":
    thrust_source_selector_node = ThrustSourceSelectorNode()
    thrust_source_selector_node.run()