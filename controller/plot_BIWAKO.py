# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 13:20:30 2019

@author: FujiiChang
"""

import world_class
import robot_class
import math
import datetime
import os
import numpy as np

now = datetime.datetime.now()
date = now.strftime("%Y_%m_%d_%H_%M_%S")

os.mkdir("../../plot_data/" + date)

# import way_point data
way_point = np.genfromtxt('./way_point/square.csv', delimiter=',', dtype='float', encoding='utf-8')
world = world_class.World(way_point)

# input tracking data
data = np.genfromtxt('./csv/square_new.csv', delimiter=',',
                  names=True, dtype=None, encoding='utf-8')

initial_pose = np.array([data[0][3], data[0][2], math.radians(data[0][4])]).T
robot = robot_class.Robot(np.array(initial_pose))

world.append(robot)
world.draw(date, 0)
print(initial_pose)

for i in range(1, len(data)):
    robot = robot_class.Robot(np.array([data[i][3], data[i][2], math.radians(data[i][4])]).T)
    world.append(robot)
    world.draw(date, str(i))
