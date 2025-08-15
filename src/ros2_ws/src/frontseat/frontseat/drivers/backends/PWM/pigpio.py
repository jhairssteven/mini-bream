import pigpio
import subprocess
from pwm_backend import PWMBackend

class PigpioPWMBackend(PWMBackend):
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            try:
                subprocess.run(['sudo', 'pigpiod'], check=True)
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise RuntimeError("Failed to connect to pigpio daemon")
            except subprocess.CalledProcessError as e:
                raise RuntimeError(f"Failed to start pigpio daemon: {e}")
                

    def set_duty_cycle(self, pin: int, frequency: int, duty_cycle: float):
        """pigpio.hardware_PWM uses duty cycle in range [0, 1e6]"""
        duty_scaled = int(duty_cycle * 10000)  # percent → parts-per-million
        self.pi.hardware_PWM(pin, frequency, duty_scaled)
