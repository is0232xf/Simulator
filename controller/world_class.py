# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:03:14 2019

@author: FujiiChang
"""

import math
import matplotlib.pyplot as plt

class World:
    def __init__(self):
        self.objects = []
        self.longitude = 135.963535
        self.latitude = 34.979778
        self.R = 6378137
        self.ey = 360/(2*math.pi*self.R)
        self.ex = 360/(2*math.pi*self.R*math.cos(self.latitude*math.pi/180))
        
    def append(self, obj):
        self.objects.append(obj)
    
    def draw(self):
        fig = plt.figure(figsize=(8 ,8))
        ax = fig.add_subplot(1, 1, 1)
        ax.set_aspect("equal")
        ax.set_xlim(self.longitude-100*self.ex, self.longitude+100*self.ex)
        ax.set_ylim(self.latitude, self.latitude+300*self.ey)
        ax.set_xlabel("X", fontsize=10)
        ax.set_ylabel("Y", fontsize=10)
        
        for obj in self.objects: obj.draw(ax)
        
        plt.show()
