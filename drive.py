import math
import numpy as np
import Adafruit_PCA9685
import math

class Drive:
    def __init__(self, config):
        self.config = config['drive']
        self.pwm = Adafruit_PCA9685.PCA9685(0x47)

    def omni(self, speed, heading, alpha):
        heading = math.radians(heading)

        desired_vector = (speed) * np.array((math.cos(heading), math.sin(heading)))

        wheels = np.radians([120, 240, 0])
        wheel_vectors = np.array((np.cos(wheels), np.sin(wheels)))
        wheel_vectors = wheel_vectors

        wheel_powers = np.dot(desired_vector, wheel_vectors)

        # max_rot = max_wheel_speed / R
        max_alpha = 1 - max(wheel_powers)
        min_alpha = -1 - min(wheel_powers)
        alpha = max(min_alpha, min(max_alpha, alpha))
        wheel_powers = wheel_powers + alpha

        self.wheels(wheel_powers)

    def wheels(self, powers):
        for wheel_num in range(3):
            power = powers[wheel_num]
            pwmVal = int(min(4095, max(0, abs(power) * 4095)))
            print("Wheel ", wheel_num, "At: ", pwmVal)
            pins = self.config['wheelPins'][wheel_num]
            if power >= 0:
                self.pwm.set_pwm(pins['fwd'], 0, pwmVal)
                self.pwm.set_pwm(pins['rev'], 0, 0)
            else:
                self.pwm.set_pwm(pins['rev'], 0, pwmVal)
                self.pwm.set_pwm(pins['fwd'], 0, 0)
