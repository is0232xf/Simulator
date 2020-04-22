# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 18:56:24 2020

@author: FujiiChang
"""
import csv
import datetime
import math
import time

import numpy as np

import disturbance_class as disturbance
import plot_BIWAKO
from controller_class import Controller
from earth_class import Earth
from robot_class import Robot

file_log = True

if file_log:
#################################### set up for file log###################################
    # obtaine date time object for file name definition
    detail = datetime.datetime.now()
    date = detail.strftime("%Y_%m_%d_%H_%M_%S")
    ground_truth_file = '../../csv_data/'+ date +'_gt.csv' # open csv file
    gps_file = '../../csv_data/'+ date +'_gps.csv' # open csv file
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
try:
    # disturbance = disturbance.disturbance()
    disturbance = np.array([[0],
                            [0],
                            [0]])
    while True:
        action = Controller.decide_next_action(Okebot)
        if action[0] == "f":
            break
        pwm = Controller.update_pwm_pulse(action)
        Okebot.update_pwm_pulse(pwm)
        Okebot.update_state(disturbance)
        target_point = Controller.next_goal
        # current_point = [Okebot.x, Okebot.y]
        Okebot.gps.update_gps_value(Okebot.x, Okebot.y)
        if file_log:
            gt_log.append([target_point[0], target_point[1],
                           Okebot.x, Okebot.y,
                           math.degrees(Okebot.yaw)])
            gps_log.append([target_point[0], target_point[1],
                            Okebot.gps.longitude, Okebot.gps.latitude,
                            math.degrees(Okebot.yaw)])
        time.sleep(0.05)
    print("Mission complete")
    if file_log:
        for gt_log_data in gt_log:
            gt_Writer.writerow(gt_log_data)
        for gps_log_data in gps_log:
            gps_Writer.writerow(gps_log_data)

        ground_truth_data.close()
        gps_data.close()
        plot_BIWAKO.make_figure(way_point_file, ground_truth_file)
        plot_BIWAKO.make_figure(way_point_file, gps_file)
except KeyboardInterrupt:
    if file_log:
        ground_truth_data.close()
        gps_data.close()
