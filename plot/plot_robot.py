# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:25:07 2019

@author: FujiiChang
"""

import world_class
import robot_class
import numpy as np

world = world_class.World(50,50)

robot1 = robot_class.Robot(np.array([0, 0, 0.0]).T)

world.append(robot1)
world.draw()

robot1.move(10, 0)
world.draw()
robot1.move(0, 20)
world.draw()
robot1.move(20, 0)
world.draw()
robot1.move(10, 0)
world.draw()