# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 16:09:20 2020

@author: FujiiChang
"""

import math

class Earth:
    def __init__(self, lat):
        self.R = 6378137
        self.ey = 360/(2*math.pi*self.R) # latitude: 1deg -> 1m
        self.ex = 360/(2*math.pi*self.R*math.cos(lat*math.pi/180)) # longitude: 1deg -> 1m

    def calc_ex(self, lat):
        self.ex = 360/(2*math.pi*self.R*math.cos(lat*math.pi/180)) # longitude: 1deg -> 1m
