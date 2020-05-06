# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 13:20:30 2019

@author: FujiiChang
"""

import world_class
import robot_class
import math
import datetime
import numpy as np

def make_figure(way_point, file, file_path):
	# import way_point data
	way_point = np.genfromtxt(way_point, delimiter=',', dtype='float', encoding='utf-8')
	world = world_class.World(way_point)

	# input tracking data
	data = np.genfromtxt(file, delimiter=',',
	                  names=True, dtype=None, encoding='utf-8')

	initial_pose = np.array([data[0][3], data[0][2], math.radians(data[0][4])]).T
	robot = robot_class.DrawRobot(np.array(initial_pose))

	world.append(robot)

	for i in range(1, len(data)):
	    robot = robot_class.DrawRobot(np.array([data[i][3], data[i][2], math.radians(data[i][4])]).T)
	    world.append(robot)
	    if i == len(data)-1:
    		world.draw(file_path)

def make_config_txt(dir_name):
    import json
    params_file = open("params.json", "r")
    params = json.load(params_file)
    config_txt = open(dir_name + "/detail.txt", "a")
    config_txt.write("tolerance: " + str(params["Controller"]["torelance"]) + "\n")
    config_txt.write("GPS\n")
    config_txt.write("error avg: " + str(params["GPS"]["e_mean"]) +  " dev: " + str(params["GPS"]["e_std_dev"]) + "\n")
    config_txt.write("Disturbance\n")
    config_txt.write("x: " + str(params["Disturbance"]["force_x"]) + " y: " + str(params["Disturbance"]["force_y"]) + "\n")

    config_txt.close()
    params_file.close()
