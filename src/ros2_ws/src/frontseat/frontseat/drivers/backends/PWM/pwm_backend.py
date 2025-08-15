class PWMBackend:
    """  Abstract PWM interface (base class) """
    def set_duty_cycle(self, duty_cycle: float):
        """Set PWM signal duty cycle.
        duty_cycle is in percentage [0.0, 100.0]
        """
        raise NotImplementedError
