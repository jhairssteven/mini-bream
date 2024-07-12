#! /usr/bin/python3
import rospy
import pigpio
from std_msgs.msg import Float32, String

class PWMMotorController:
    def __init__(self, pin=None, motor_name=None):
        if pin is None:
            raise ValueError("Missing required argument: A 'pin' number must be defined to use PWM motor controller")
        elif motor_name is None:
            raise ValueError("Missing required argument: A 'motor_name' name must be defined to use PWM motor controller")
        
        self.pin = pin
        self.motor_name = motor_name
        self.zero_throttle_us = 1500
        self.freq = 340 # Set the default base PWM Frequency in [Hz] (based on ESC specifications)
        import subprocess
        try:
            self.piGPIO = pigpio.pi()
            if not self.piGPIO.connected:
                try:
                    # initialize pigpio daemon
                    command = ['sudo', 'pigpiod']
                    # Execute the command and capture output
                    result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    rospy.loginfo("STDOUT:", result.stdout)
                    rospy.loginfo("STDERR:", result.stderr)                
                except subprocess.CalledProcessError as e:
                    rospy.logerr(f"Command failed with return code {e.returncode}")
                    rospy.logerr("Error output:", e.stderr)
                    raise Exception
        except Exception as e:
            rospy.logerr("Couldn't initialize pigpio library connection")
            #exit()

        self.setThrust(0) # Must set thurst to zero before using motor (based on ESC specifications)

    def getGPIOInstance(self):
        import subprocess
        try:
            piGPIO = pigpio.pi()
            if not self.piGPIO.connected:
                try:
                    # initialize pigpio daemon
                    command = ['sudo', 'pigpiod']
                    # Execute the command and capture output
                    result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    rospy.loginfo("STDOUT:", result.stdout)
                    rospy.loginfo("STDERR:", result.stderr)
                    return piGPIO
                
                except subprocess.CalledProcessError as e:
                    rospy.logerr(f"Command failed with return code {e.returncode}")
                    rospy.logerr("Error output:", e.stderr)
                    raise Exception
        except Exception as e:
            rospy.logerr("Couldn't initialize pigpio library connection")
            exit()

    def setDutyCycle(self, dc):
        '''Input:
                dc: Desired motor dutycycle [0, 100]
        '''
        pin = self.pin
        freq = int(self.freq)
        duty_cycle = int(dc*10000)
        
        self.piGPIO.hardware_PWM(pin, freq, duty_cycle)
        #rospy.loginfo("[PWM] pin: {}, freq: {}, dc: {}".format(pin, freq, duty_cycle))
    
    def getFromControlToMicroSeconds(self, thrust):
        '''
        Args:
            thrust (float): Motor thrust. Range between [-1, 0] for backward acceleration and [0, 1] for forward.
        Returns:
            Equivalent PWM signal pulse width in microseconds
        '''
        # Limit thrust output to prevent motor overload (security factor needs calibration)
        #thrust = min(max(thrust, self.allowed_trust_security_factor), self.allowed_trust_security_factor)
        # y = (400*thrust + 1500) # single function for all [-1, 1] (6% controlability loss around zero thrust with deadband)

        pw_us = self.zero_throttle_us
        if (thrust < 0): # Reverse [-1, 0]
            if thrust < -1: thrust = -1
            pw_us = (375*thrust + 1475) #TODO: make parametric with self.max_reverse_us
        elif (thrust > 0): # Forward case [0, 1]
            if thrust > 1: thrust = 1
            pw_us = (375*thrust + 1525) #TODO: make parametric with self.max_forward_us
        
        pw_us_lim = self.thrust_limiter(pw_us)
        return pw_us_lim
    
    def thrust_limiter(self, pw_us):
        '''Limit thrust output to prevent motor overload (security factor needs calibration)'''
        max_limited_reverse = 1381.25
        max_limited_forward = 1618.75
        if (self.pin == 18): #TODO: left mtr power calibration
            max_limited_forward = 1610 #1600 #1618.75*0.8
            max_limited_reverse = 1380
        min_limited_reverse = 1467.5 #1475-(1532.5-1525)
        min_limited_forward = 1532.5
        
        if pw_us < max_limited_reverse:
            pw_us = max_limited_reverse
        elif pw_us > max_limited_forward:
            pw_us = max_limited_forward
        elif pw_us > min_limited_reverse and pw_us < self.zero_throttle_us:
            pw_us = min_limited_reverse
        elif pw_us < min_limited_forward and pw_us > self.zero_throttle_us:
            pw_us = min_limited_forward
        
        #print("pw_us: {}, max_limited_reverse: {}, max_limited_forward: {}".format(pw_us, max_limited_reverse, max_limited_forward))
        #return min(max(pw_us, max_limited_reverse), max_limited_forward)
        return pw_us
    
    def getDutyCyclefromPulseWidth(self, pw):
        ''' Calculates the equivalent duty cycle in percentage (%) [0.0, 100.0]
          given a signal pulse width (:param us) in microseconds
        '''
        T = 1 / self.freq * 10**6 # (Signal period in us)
        return pw/T * 100.0

    def setThrust(self, thrust):
        """This functional will simply command the motor controller to move the motor
        in a specified direction at some pulse width
        direction = forward or reverse with a 1 being forward and backwards being 0
        mtrspeed = desired motor output on a scale from full reverse (-1) to full (1)"""
        self.thrust = thrust
        self.thrust_us = self.getFromControlToMicroSeconds(thrust=thrust)
        self.dc = self.getDutyCyclefromPulseWidth(pw=self.thrust_us)
        self.setDutyCycle(self.dc)

        #rospy.loginfo(self.__techs())

    def __techs(self):
        """ Gives motor last commanded thrust with equivalent Pulse witdh and duty cycle logs """
        return '[{}] thrust:{}, pw (us): {}, dc: {}'.format(self.motor_name, self.thrust, self.thrust_us, self.dc)

""" class MotorControllerNode:
    def __init__(self, node_name='motor_controller'):
        self.node_name = node_name
        self.left_motor_pwm_controller = PWMMotorController(pin=18, motor_name="left_mtr")
        self.right_motor_pwm_controller = PWMMotorController(pin=13, motor_name="right_mtr")
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
    motor_controller_node = MotorControllerNode(node_name='motor_controller')
    motor_controller_node.run() """