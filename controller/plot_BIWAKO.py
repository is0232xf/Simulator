# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 13:20:30 2019

@author: FujiiChang
"""

import world_class
import robot_class
import math
import numpy as np


way_point = np.array([[34.982187, 135.963695], [34.982192, 135.963596], [34.982118, 135.963598],
                      [34.982118, 135.963698]])
"""
way_point = np.array([[34.980779, 135.963660], [34.981122, 135.963526], [34.981425, 135.963706], [34.981598, 135.963561],
                      [34.981598, 135.963561], [34.981095, 135.963695],[34.980350, 135.963502]])
"""
world = world_class.World(way_point)

data = np.genfromtxt('./csv/test6.csv', delimiter=',',
                  names=True, dtype=None, encoding='utf-8')

initial_pose = np.array([data[0][3], data[0][2], math.radians(data[0][4])]).T
robot = robot_class.Robot(np.array(initial_pose))

world.append(robot)
world.draw()
print(initial_pose)

for i in range(1, len(data)):
    robot = robot_class.Robot(np.array([data[i][3], data[i][2], math.radians(data[i][4])]).T)
    world.append(robot)
    world.draw()
