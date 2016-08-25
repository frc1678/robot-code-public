import numpy as np
import math

"""
A simple trapezoidal motion profile class to test controllers with. This
class does not deal with all of the corner cases dealt with in the C++ version.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class trapezoidal_profile:
    def __init__(self, distance, max_speed, max_acceleration):
        self.total_distance = distance
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration

        self.acceleration_time = self.max_speed / self.max_acceleration

        if self.acceleration_time * self.acceleration_time * self.max_acceleration > self.total_distance:
            self.acceleration_time = math.sqrt(self.total_distance / self.max_acceleration)
            self.max_speed = self.acceleration_time * self.max_acceleration

        self.acceleration_distance = self.max_speed * self.acceleration_time / 2.0
        self.full_speed_distance = self.total_distance - 2.0 * self.acceleration_distance
        self.full_speed_time = self.full_speed_distance / self.max_speed

        self.total_time = self.acceleration_time * 2.0 + self.full_speed_time


    def distance(self, t):
        if t < self.acceleration_time:
            return (t / self.acceleration_time) ** 2.0 * self.acceleration_distance
        elif t < self.acceleration_time + self.full_speed_time:
            return (t - self.acceleration_time) * self.max_speed + self.acceleration_distance
        elif t < self.total_time:
            return self.total_distance - ((self.total_time - t) / self.acceleration_time) ** 2.0 * self.acceleration_distance
        else:
            return self.total_distance

    def velocity(self, t):
        if t < self.acceleration_time:
            return (t / self.acceleration_time) * self.max_speed
        elif t < self.acceleration_time + self.full_speed_time:
            return self.max_speed
        elif t < self.total_time:
            return ((self.total_time - t) / self.acceleration_time) * self.max_speed
        else:
            return 0
