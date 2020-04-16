import math
import random

import numpy as np
from earth_class import Earth

class GPS:
    def __init__(self):
        self.e_mean = 1.0
        self.e_std_dev = 1.0

    def return_gps_value(self, longitude, latitude):
        earth = Earth(latitude)
        e = np.random.normal(self.e_mean, self.e_std_dev)
        theta = random.uniform(-math.pi, math.pi)
        longitude = longitude + e*math.cos(theta)*earth.ex
        latitude = latitude + e*math.sin(theta)*earth.ey
        return longitude, latitude
