import Jetson.GPIO as jGPIO
from pwm_backend import PWMBackend

class JetsonPWMBackend(PWMBackend):
    def __init__(self, pin: int, frequency: int, start_duty_cycle: float):
        self.pwm_handler = self.get_pwm_handler_for(pin, frequency, start_duty_cycle)
    
    def get_pwm_handler_for(self, pin, hz, start_duty_cycle):
        try:
            jGPIO.setup(pin, jGPIO.OUT)
            pwm = jGPIO.PWM(pin, hz)
            pwm.start(start_duty_cycle)
            return pwm
        except Exception as e:
            raise RuntimeError(f"Failed to initialize PWM on pin {pin} with frequency {hz} Hz: {e}")

    
    def set_duty_cycle(self, duty_cycle: float):
        self.pwm_handler.ChangeDutyCycle(duty_cycle)
