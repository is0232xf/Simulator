# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:56:19 2019

@author: FujiiChang
"""

import math

class Earth:
    def __init__(self, latitude):
        R = 6378137 # Earth radius WGS84
        ey = 360/(2*math.pi*R) # latitude: 1deg -> 1m
        ex = 360/(2*math.pi*R*math.cos(latitude*math.pi/180)) # longitude: 1deg -> 1m
