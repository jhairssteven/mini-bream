import rospy
import pigpio # This library uses hardware_pwm which is more efficient than software_pwm used on Rpi.GPIO library

class PWMMotorController:
    def __init__(self, pin=None, motor_name=None):
        if pin is None:
            raise ValueError("Missing required argument: A 'GPIO_pin' number must be defined to use PWM motor controller")
        elif motor_name is None:
            raise ValueError("Missing required argument: A 'motor_name' name must be defined to use PWM motor controller")
        
        self.pin = pin
        self.motor_name = motor_name
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

    def setDutyCycle(self, dc):
        '''Input:
                dc: Desired motor dutycycle [0, 100]
        '''
        pin = self.pin
        freq = int(self.freq)
        duty_cycle = int(dc*10000)
        
        self.piGPIO.hardware_PWM(pin, freq, duty_cycle)
        #rospy.loginfo("[{}] pin: {}, freq: {}, dc: {}, us: {}".format(self.motor_name, pin, freq, dc, self.thrust_us))

    def getEquivalentPulseWidth(self, thrust, backwards_thrust_range=(-1, 0), forwards_thrust_range=(0, 1), backwards_pwm_limits=(1000, 1440), neutral_pwm=(0, 1500), forwards_pwm_limits=(1560, 2000)):
        '''
        Args:
            thrust (float): Motor thrust. Range between [-1, 0] for backward acceleration and [0, 1] for forward.
            *_pwm_limits: PWM controller-specific limits in microseconds
        Returns:
            Equivalent PWM signal pulse width in microseconds
        '''
        def linear_mapping(x, x_range, y_range):
            """ Gets a linear mapping of x_range to y_range in the form [x_min, x_max] -> [y_min, y_max] """
            x_min, x_max = x_range
            y_min, y_max = y_range
            # Calculate the slope (m) and intercept (b) for the linear equation y = mx + b
            m = (y_max - y_min) / (x_max - x_min)
            b = y_min - m * x_min
            return m * x + b
        
        if backwards_thrust_range[0] <= thrust < backwards_thrust_range[1]:
            return linear_mapping(thrust, backwards_thrust_range, backwards_pwm_limits)
        elif thrust == neutral_pwm[0]:
            return neutral_pwm[1]
        elif forwards_thrust_range[0] < thrust <= forwards_thrust_range[1]:
            return linear_mapping(thrust, forwards_thrust_range, forwards_pwm_limits)
        else:
            raise ValueError("'thrust' is out of the defined range")
        
    def getDutyCyclefromPulseWidth(self, pw):
        ''' Calculates the equivalent duty cycle in percentage (%) [0.0, 100.0]
          given a signal pulse width (:param us) in microseconds
        '''
        T = 1 / self.freq * 10**6 # (Signal period in us)
        return pw/T * 100.0
    
    def setThrust(self, thrust):
        """Move motor at specified thrust.
        @param 'thrust': Must be in range [-1, 1], for fully reverse and fully forward control respectively.
        """
        sensibility = 0.7 # This limits the actual motor-thrust to prevent abrupt velocities or current overshoots
        self.thrust = thrust*sensibility
        self.thrust_us = self.getEquivalentPulseWidth(thrust=self.thrust)
        self.dc = self.getDutyCyclefromPulseWidth(pw=self.thrust_us)
        self.setDutyCycle(self.dc)

        #rospy.loginfo(self.__techs())

    def __techs(self):
        """ Gives motor last commanded thrust with equivalent Pulse witdh and duty cycle logs """
        return '[{}] thrust:{}, pw (us): {}, dc: {}'.format(self.motor_name, self.thrust, self.thrust_us, self.dc)
