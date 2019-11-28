# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:03:14 2019

@author: FujiiChang
"""

import math
import numpy as np
import matplotlib.pyplot as plt

class World:
    def __init__(self, points):
        self.objects = []
        self.points = points
        self.R = 6378137
        self.ey = 360/(2*math.pi*self.R)
        self.ex = 360/(2*math.pi*self.R*math.cos(np.mean(self.points[0,:])*math.pi/180))
        print("mean latitude: ", np.mean(self.points[0,:]))
        
    def append(self, obj):
        self.objects.append(obj)
    
    def draw(self, date, name):
        fig = plt.figure(figsize=(8 ,6))
        plt.grid(True)
        ax = fig.add_subplot(1, 1, 1)
        ax.set_aspect("equal")        
        ax.set_xlim(min(self.points[:,1])-0.6*self.ex, max(self.points[:,1])+0.6*self.ex)
        ax.set_ylim(min(self.points[:,0])-9*self.ey, max(self.points[:,0])+9*self.ey)
        ax.set_xlabel("X", fontsize=10)
        ax.set_ylabel("Y", fontsize=10)
        self.plot_target_point(self.points, ax)
        
        for obj in self.objects:
            obj.draw(ax)
        
        plt.savefig("../../plot_data/" + str(date) + "/" + str(name) + ".png")
        plt.show()
        plt.clf()
    
    def plot_target_point(self, points, ax):
        for point in points:
            ax.scatter(point[1], point[0])
