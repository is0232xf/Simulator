# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:15:26 2019

@author: FujiiChang
"""

import math
import matplotlib.patches as patches

R = 6378137 # Earth radius WGS84

class Robot:
    def __init__(self, pose, color="black"):
        self.pose = pose
        self.r = 5
        self.color = color
        
    def move(self, power, omega):
        """
        ey = 360/(2*math.pi*R) # latitude: 1deg -> 1m
        ex = 360/(2*math.pi*R*math.cos(self.pose[0]*math.pi/180)) # longitude: 1deg -> 1m
        """
        t_x =  math.cos(self.pose[2])*power
        t_y = math.sin(self.pose[2])*power
        theta = math.radians(omega)
        print("omega: ", omega)
        print("theta: ", theta)
        
        print("current pose: ", self.pose)
        self.pose[0] = self.pose[0] + t_x
        self.pose[1] = self.pose[1] + t_y
        self.pose[2] = self.pose[2] + theta
        print("move to: ", self.pose)
        
    def draw(self, ax):
        x, y, theta = self.pose
        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        ax.plot([x, xn], [y, yn], color=self.color)
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)
        ax.add_patch(c)
    