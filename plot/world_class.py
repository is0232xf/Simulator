# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:03:14 2019

@author: FujiiChang
"""

import matplotlib.pyplot as plt

class World:
    def __init__(self, xlim, ylim):
        self.objects = []
        self.xlim = xlim
        self.ylim = ylim
        
    def append(self, obj):
        self.objects.append(obj)
    
    def draw(self):
        fig = plt.figure(figsize=(4 ,4))
        ax = fig.add_subplot(1, 1, 1)
        ax.set_aspect("equal")
        ax.set_xlim(-1*self.xlim, self.xlim)
        ax.set_ylim(-1*self.ylim, self.ylim)
        ax.set_xlabel("X", fontsize=20)
        ax.set_ylabel("Y", fontsize=20)
        
        for obj in self.objects: obj.draw(ax)
        
        plt.show()
