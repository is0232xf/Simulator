# -*- coding: utf-8 -*-
"""
Created on Fri Dec 13 21:26:11 2019

@author: User
"""

import math
import json
import calculate_angle
import numpy as np

class Disturbance:

    def __init__(self):
        self.params_file = open("params.json", "r")
        self.params = json.load(self.params_file) 
        self.wave = np.array([0.0, 0.0])
        self.window = np.array([0.0, 0.0])
        self.force_x = self.params["Disturbance"]["force_x"]
        self.force_y = self.params["Disturbance"]["force_y"]

    def __del__(self):
        self.params_file.close()

    def shift_wave_term(self):
        wave_force = 0.5 * np.random.normal()
        wave_direction = calculate_angle.limit_angle(2 * math.pi * np.random.normal())
        self.wave = np.array([wave_force, wave_direction])

    def shift_window_term(self):
        window_force = 0.5 * np.random.normal()
        window_direction = calculate_angle.limit_angle(2 * math.pi * np.random.normal())
        self.window = np.array([window_force, window_direction])

    def change_disturbance_force(self):
       self.force_x = self.wave[0]*math.cos(self.wave[1]) + self.window[0]*math.cos(self.window[1])
       self.force_y = self.wave[0]*math.sin(self.wave[1]) + self.window[0]*math.sin(self.window[1])
       