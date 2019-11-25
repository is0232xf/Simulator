# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 13:20:30 2019

@author: FujiiChang
"""

import world_class
import robot_class
import math
import numpy as np

world = world_class.World()

data = np.genfromtxt('test.csv', delimiter=',',
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
