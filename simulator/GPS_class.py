import math
import random

import numpy as np
from earth_class import Earth

class GPS:
    def __init__(self, longitude, latitude):
        # Error model params
        self.longitude = longitude
        self.latitude = latitude
        self.e_mean = 2.0
        self.e_std_dev = 1.0

    def update_gps_value(self, longitude, latitude):
        earth = Earth(latitude)
        e = np.random.normal(self.e_mean, self.e_std_dev)
        theta = random.uniform(-math.pi, math.pi)
        self.longitude = longitude + e*math.cos(theta)*earth.ex
        self.latitude = latitude + e*math.sin(theta)*earth.ey
