from math import *
import numpy as np

class Omni:
    def __init__(self):
        self.wheels = [[cos(radians(x)), sin(radians(x))] for x in [0, 120, 240]]

    def get_speeds(self, vec, polar = False):
        if polar:
            speed = vec[0]
            r_angle = radians(vec[1])
            vec = [speed * cos(r_angle), speed * sin(r_angle)]

        return tuple(np.reshape(np.dot(self.wheels, vec), -1))