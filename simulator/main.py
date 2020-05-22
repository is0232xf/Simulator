# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 18:56:24 2020

@author: FujiiChang
"""
import csv
import datetime
import math
import os
import time
import json

import numpy as np

from disturbance import Disturbance
import plot_BIWAKO
from controller import Controller
from earth import Earth
from robot import Robot

params_file = open("params.json", "r")
params = json.load(params_file)
file_log = params["Log"]["file_log"]

if file_log:
#################################### set up for file log###################################
    # obtaine date time object for file name definition
    detail = datetime.datetime.now()
    date = detail.strftime("%Y_%m_%d_%H_%M_%S")
    dir_name = "./data/plot_data/" + date 
    os.mkdir(dir_name)
    ground_truth_file = './data/csv/'+ date +'_gt.csv' # open csv file
    gps_file = './data/csv/'+ date +'_gps.csv' # open csv file
    ground_truth_data = open(ground_truth_file, 'a', newline='')
    gps_data = open(gps_file, 'a', newline='')
    gt_Writer = csv.writer(ground_truth_data)
    gps_Writer = csv.writer(gps_data)
    gt_log = []
    gps_log = []
###########################################################################################

way_point_file = './way_point/biwako.csv' # read waypoint file (csv)
way_point = np.genfromtxt(way_point_file,
                          delimiter=',',
                          dtype='float',
                          encoding='utf-8')

Controller = Controller(way_point)
pose = np.array([Controller.way_point[0][0], Controller.way_point[0][1], math.radians(90)]) # initialize the robot position
Okebot = Robot(pose)
# define variables

Earth =  Earth(Okebot.y)
###########################################################################################

# input: driving force of the robot
# output: drag force
"""
def calc_drag(F):
    D_f = 0 # Drag effected by flow
    D_w = 0 # Drag effected by waves
    D = np.array([D_x],  # Drag forces which subjects to the robot
                 [D_y],
                 [D_r])
    return D
"""

def data_log(Disturbance, Controller, Robot):
    for gt_log_data in gt_log:
        gt_Writer.writerow(gt_log_data)
    for gps_log_data in gps_log:
        gps_Writer.writerow(gps_log_data)

    ground_truth_data.close()
    gps_data.close()

    file_path_gt = dir_name + "/groud_truth/"
    file_path_gps = dir_name + "/gps/"
    os.mkdir(file_path_gt)
    os.mkdir(file_path_gps)
    plot_BIWAKO.make_figure(way_point_file, ground_truth_file, file_path_gt)
    plot_BIWAKO.make_figure(way_point_file, gps_file, file_path_gps)

    plot_BIWAKO.make_config_txt(dir_name)
    del Disturbance
    del Controller
    del Robot

if __name__ == "__main__":
    try:
        Disturbance = Disturbance()
        disturbance_force = np.array([[Disturbance.force_x],
                                    [Disturbance.force_y],
                                    [0.0]])
        while True:
            action = Controller.decide_next_action(Okebot)
            if action[0] == "f":
                break
            pwm = Controller.update_pwm_pulse(action)
            Okebot.update_pwm_pulse(pwm)
            Okebot.update_state(disturbance_force)
            target_point = Controller.next_goal
            Okebot.gps.update_gps_value(Okebot.x, Okebot.y)
            if file_log:
                gt_log.append([target_point[0], target_point[1],
                            Okebot.x, Okebot.y,
                            math.degrees(Okebot.yaw)])
                gps_log.append([target_point[0], target_point[1],
                                Okebot.gps.longitude, Okebot.gps.latitude,
                                math.degrees(Okebot.yaw)])
        print("Mission complete")
        if file_log:
            data_log(Disturbance, Controller, Okebot)
    except KeyboardInterrupt:
        print("Keyboard Interrupt!")
        if file_log:
            data_log(Disturbance, Controller, Okebot)