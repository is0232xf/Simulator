# -*- coding: utf-8 -*-
"""
Created on Fri Dec 13 21:26:11 2019

@author: User
"""

import math
import numpy as np

class disturbnce:

    def __init__(self):
       self.wave = 0.0
       self.window = 0.0
       self.wave_direction = 0.0
       self.window_direction = 0.0

    def shift_wave_term(self):
        self.wave = 3 * np.random.normal()
        self.wave_direction = 2 * math.pi() * np.random.normal()

    def shift_window_term(self):
        self.window = 1 * np.random.normal()
        self.window_direction = 2 * math.pi() * np.random.normal()
