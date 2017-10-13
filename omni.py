import math
import numpy as np
import Adafruit_PCA9685

class Drive:
    def __init__(self, config):
        self.config = config['drive']
        self.pwm = Adafruit_PCA9685.PCA9685(0x47)

    def omni(self, speed, heading):

        R = self.config['wheelR']
        max_wheel_speed = self.config['wheelSpeed'] * math.pi * self.config['wheelDiameter']
        desired_vector = (speed / max_wheel_speed) * np.array((math.cos(heading), math.sin(heading)))

        wheels = np.radians([120, 240, 0])
        wheel_vectors = np.array((np.cos(wheels), np.sin(wheels)))
        wheel_vectors = wheel_vectors
        self.wheels(np.dot(desired_vector, wheel_vectors))

    def wheels(self, powers):
        for wheel_num in range(3):
            power = powers[wheel_num]
            pwmVal = int(min(4095, max(0, abs(power) * 4095)))
            print("Wheel ", wheel_num, "At: ", pwmVal)
            pins = self.config['wheelPins'][wheel_num]
            if power >= 0:
                self.pwm.set_pwm(pins['fwd'], 0, pwmVal)
            else:
                self.pwm.set_pwm(pins['rev'], 0, pwmVal)
