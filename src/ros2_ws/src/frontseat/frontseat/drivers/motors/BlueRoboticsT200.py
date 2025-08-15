#! /usr/bin/python3
from frontseat.drivers.backends.PWM.jetsonGPIO import JetsonPWMBackend

class BlueRoboticsT200:
    def __init__(self, pin: int = None, motor_name: str = None):

        if pin is None:
            raise ValueError("A 'pin' number must be provided")
        if not motor_name:
            raise ValueError("A 'motor_name' must be provided")

        self.pin = pin
        self.freq = 340  # Hz, based on ESC specs
        # Must set thurst to zero before using motor (based on ESC specifications)
        self.zero_throttle_dc = self.getDutyCycleFromThrust(0.0)
        self.pwm_backend = JetsonPWMBackend(self.pin, self.freq, start_duty_cycle=self.zero_throttle_dc)
        self.motor_name = motor_name
    
    def getEquivalentPulseWidth(self, thrust, backwards_thrust_range=(-1, 0), forwards_thrust_range=(0, 1), backwards_pwm_limits=(1000, 1440), neutral_pwm=(0, 1500), forwards_pwm_limits=(1560, 2000)):
        '''Convert [-1, 1] thrust to PWM pulse width in µs.
        Args:
            thrust (float): Motor thrust. Range between [-1, 0] for backward acceleration and [0, 1] for forward.
            *_pwm_limits: PWM controller-specific limits in microseconds
        Returns:
            Equivalent PWM signal pulse width in microseconds
        '''

        # y = (400*thrust + 1500) # single function for all [-1, 1] (6% controlability loss around zero thrust with deadband)

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
        elif forwards_thrust_range[0] < thrust <= forwards_thrust_range[1]:
            return linear_mapping(thrust, forwards_thrust_range, forwards_pwm_limits)
        elif thrust == neutral_pwm[0]:
            return neutral_pwm[1]
        else:
            raise ValueError("'thrust' is out of the defined range")

    def getDutyCyclefromPulseWidth(self, pw):
        """Convert µs pulse width to duty cycle percentage (0.0, 100.0%)."""
        T = 1 / self.freq * 10**6 # (Signal period in us)
        return pw / T * 100.0

    def getDutyCycleFromThrust(self, thrust):
        thrust_us = self.getEquivalentPulseWidth(thrust=thrust)
        dc = self.getDutyCyclefromPulseWidth(pw=thrust_us)
        return dc
    
    def setThrust(self, thrust):
        """Command the motor with thrust in [-1, 1]."""
        sensibility = 0.7 # This limits the actual motor-thrust to prevent abrupt velocities or current overshoots
        self.dc = self.getDutyCycleFromThrust(thrust*sensibility)
        self.pwm_backend.set_duty_cycle(self.dc)

    def __techs(self):
        """ Gives motor last commanded thrust with equivalent Pulse witdh and duty cycle logs """
        return '[{}] thrust:{}, pw (us): {}, dc: {}'.format(self.motor_name, self.thrust, self.thrust_us, self.dc)
