# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import math
import numpy as np
import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(0x47)

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 2048  # Max pulse length out of 4096
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:

    maxHz = 276 / 60.0
    maxWheelSpeed = maxHz * math.pi * 0.067
    desiredSpeed = float(sys.argv[1])
    desiredHeading = math.radians(float(sys.argv[2]))
    desiredVector = (desiredSpeed / maxWheelSpeed) * np.array((math.cos(desiredHeading), math.sin(desiredHeading)))

    R = 0.09

    maxWheelSpeed = 276 / 60.0

    wheels = np.radians([120, 240, 0])
    wheelVectors = np.array((np.cos(wheels), np.sin(wheels)))
    wheelVectors = wheelVectors
    wheelPowers = np.dot(desiredVector, wheelVectors)

    print(wheelPowers)

    wheelPins = ((2, 1), (14, 15), (13, 12))

    for wheelNum in range(3):
        power = wheelPowers[wheelNum]
        pwmVal = int(min(4095, max(0, abs(power) * 4095)))
        print("Wheel ", wheelNum, "At: ", pwmVal)
        fwdPin, revPin = wheelPins[wheelNum]
        if power >= 0:
            pwm.set_pwm(fwdPin, 0, pwmVal)
        else:
            print("Reverse")
            pwm.set_pwm(revPin, 0, pwmVal)


    time.sleep(2)

    for wheelPinSet in wheelPins:
        for pin in wheelPinSet:
            pwm.set_pwm(pin, 0, 0)

    time.sleep(5)
