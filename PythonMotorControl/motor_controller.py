from communicator import Communicator
import consts
import struct

class MotorController:

    def __init__(self, c):
        self.comm = c

    def set_diff(self, average_speed, steering):
        """
        Set the motors running through differential steering parameters
        :param average_speed: From -0.5 to 0.5, average forward speed excluding steering
        :param steering: From -1.0 to 1.0, steering from full left to full right
        :return:
        """
        steering = self.clamp(steering, -1, 1)
        average_speed = self.clamp(average_speed, -0.5, 0.5)
        if average_speed == 0:
            left = average_speed  + steering
            right = average_speed - steering
        else:
            left = average_speed  * (1 + steering)
            right = average_speed * (1 - steering)
            
        self.set_motors(left, right)
    
    def set_motors(self, left, right):
        """
        Set motor speeds and push to the arduino over serial
        :param left: Float from -1 to 1 (reverse, forward) for the left motor speed
        :param right: Float from -1 to 1 (reverse, forward) for the right motor speed
        :return: JACK SHIT
        """
        pwm_left = self.norm_to_PWM(left)
        pwm_right = self.norm_to_PWM(right)
        
        rev_left = left < 0
        rev_right = right < 0
        
        print("Left: ({}, {}) - Right: ({}, {})".format(pwm_left, rev_left, pwm_right, rev_right))
        
        bytes_left = struct.pack("<BBB", consts.HDR_LEFTMOTOR, rev_left, pwm_left)
        bytes_right = struct.pack("<BBB", consts.HDR_RIGHTMOTOR, rev_right, pwm_right)

        self.comm.write(bytes_left)
        self.comm.write(bytes_right)
                
    def clamp(self, num, mn, mx):
        """
        Clamp (limit) a number between two other numbers. Convenience function
        :param num: Number to clamp
        :param mn: Minimum bound for clamping
        :param mx: Maximum bound for clamping
        :return: Clamped number
        """
        return max(min(mx, num), mn)
                
    def norm_to_PWM(self, norm):
        """
        Change a float into a PWM value usable by the Arduino, used for motor speeds
        :param norm: Normalised value so that abs(norm) is between 0 and 1
        :return: The PWM value (uint8) between 0-255
        """
        return self.clamp(abs(int(round(norm * consts.MAX_PWM))), 0, 255)