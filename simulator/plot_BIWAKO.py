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

def make_figure(detail_data, way_point, file, file_path):
	now = datetime.datetime.now()
	date = now.strftime("%Y_%m_%d_%H_%M_%S")
	dir_name = "../../plot_data/" + date 
	os.mkdir(dir_name)
	detail_file = open(dir_name + "/detail.txt", "a")
	detail_file.write("tolerance: " + str(detail_data[0]) + "\n")
	detail_file.write("GPS\n")
	detail_file.write("error avg: " + str(detail_data[1]) +  "dev: " + str(detail_data[2]) + "\n")
	detail_file.write("Disturbance\n")
	detail_file.write("x: " + str(detail_data[3]) + "y: " + str(detail_data[4]) + "\n")

	# import way_point data
	way_point = np.genfromtxt(way_point, delimiter=',', dtype='float', encoding='utf-8')
	world = world_class.World(way_point)

	# input tracking data
	data = np.genfromtxt(file, delimiter=',',
	                  names=True, dtype=None, encoding='utf-8')

	initial_pose = np.array([data[0][3], data[0][2], math.radians(data[0][4])]).T
	robot = robot_class.DrawRobot(np.array(initial_pose))

	world.append(robot)
	# world.draw(date, 0)

	for i in range(1, len(data)):
	    robot = robot_class.DrawRobot(np.array([data[i][3], data[i][2], math.radians(data[i][4])]).T)
	    world.append(robot)
	    if i == len(data)-1:
    		world.draw(date, file_path)
