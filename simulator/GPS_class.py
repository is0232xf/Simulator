import math
import json
import random

import numpy as np
from earth_class import Earth

class GPS:
    def __init__(self, longitude, latitude):
        self.params_file = open("params.json", "r")
        self.params = json.load(self.params_file) 
        # Error model params
        self.longitude = longitude
        self.latitude = latitude
        self.e_mean = self.params["GPS"]["e_mean"]
        self.e_std_dev = self.params["GPS"]["e_std_dev"]

    def __del__(self):
        self.params_file.close()

    def update_gps_value(self, longitude, latitude):
        earth = Earth(latitude)
        e = np.random.normal(self.e_mean, self.e_std_dev)
        theta = random.uniform(-math.pi, math.pi)
        self.longitude = longitude + e*math.cos(theta)*earth.ex
        self.latitude = latitude + e*math.sin(theta)*earth.ey
