# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 18:56:24 2020

@author: FujiiChang
"""

# mainを壊すのが怖いためクローンを改造していく

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

file_log = False

if file_log:
    #################################### set up ###############################################
    # obtaine date time object for file name definition
    detail = datetime.datetime.now()
    date = detail.strftime("%Y_%m_%d_%H_%M_%S")
    file_name = '../../csv_data/'+ date +'.csv' # open csv file
    file = open(file_name, 'a', newline='')
    csvWriter = csv.writer(file)

way_point_file = './way_point/star.csv' # read waypoint file (csv)
way_point = np.genfromtxt(way_point_file,
                          delimiter=',',
                          dtype='float',
                          encoding='utf-8')
Controller = Controller(way_point)
pose = np.array([Controller.way_point[0][0], Controller.way_point[0][1], math.radians(90)]) # initialize the robot position
print("pose: ", pose)
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
   #  disturbance = disturbance.disturbance()
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
        current_point = [Okebot.x, Okebot.y]
        if file_log:
            csvWriter.writerow([target_point[0], target_point[1],
                                current_point[0], current_point[1],
                                math.degrees(Okebot.yaw)])
        time.sleep(0.05)
    print("Mission complete")
    if file_log:
        file.close()
        plot_BIWAKO.make_figure(way_point_file, file_name)
except KeyboardInterrupt:
    if file_log:
        file.close()
