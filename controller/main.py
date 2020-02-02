0.# -*- coding: utf-8 -*-
"""
Created on Thu Nov  7 16:54:01 2019

@author: FujiiChang
"""

import csv
import datetime
import time
import math
import calculate_angle
import numpy as np
import calculate_degree as cal_deg
import disturbance_class as disturbance
from robot_class import Robot
from geopy.distance import geodesic
from kinematic_control import Motor

#################################### set up ###############################################
# define variables
theta = 34.9820933 # latitude
earth_R = 6378137 # Earth radius WGS84
ey = 360/(2*math.pi*earth_R) # latitude: 1deg -> 1m
ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # longitude: 1deg -> 1m

# obtaine date time object for file name definition
detail = datetime.datetime.now()
date = detail.strftime("%Y_%m_%d_%H_%M_%S")
# open csv file
file = open('../../csv_data/'+ date +'.csv', 'a', newline='')
csvWriter = csv.writer(file)
# set up vehicle
print("waiting connection")

# read waypoint file (csv)
way_point = np.genfromtxt('./way_point/square.csv', delimiter=',', dtype='float', encoding='utf-8')

way_point_num = 0
target_point = way_point[way_point_num] # the next target point

pose = np.array([way_point[0][0], way_point[0][1], math.radians(90)]) # initialize the robot position
Okebot = Robot(pose)

pwm_initial = 1500
voltage = 11.1
T1 = Motor(pwm_initial, voltage)
T2 = Motor(pwm_initial, voltage)
T3 = Motor(pwm_initial, voltage)
T4 = Motor(pwm_initial, voltage)
###########################################################################################

# input: driving force of the robot
# output: drag force
def calc_drag(F):
    D_f = 0 # Drag effected by flow
    D_w = 0 # Drag effected by waves
   
    D = np.array([D_x],  # Drag forces which subjects to the robot
                 [D_y],
                 [D_r])
    return D

try:
    disturbance = disturbance.disturbance()
    
    while True:
        current_point = np.array([Okebot.x, Okebot.y])
        current_yaw = Okebot.yaw
        diff_distance = geodesic(current_point, target_point).m
        # check distance between current and target
        if abs(diff_distance) < 0.5:
            print(diff_distance)
            print("complete mission ", way_point_num)
            way_point_num = way_point_num + 1
            if way_point_num >= len(way_point):
                break
            target_point = way_point[way_point_num]
            print("change waypoint")
            print("next way point: ", target_point)
        # when the device has not received
        else:
            target_direction = math.degrees(calculate_angle.limit_angle(
                    math.radians(cal_deg.calculate_bearing(current_point, target_point))))
            current_yaw = math.degrees(current_yaw)
            diff_deg =  math.degrees(calculate_angle.limit_angle(math.radians(target_direction - current_yaw)))
            if abs(diff_deg) < 2:
                print(diff_distance)
                Okebot.x = Okebot.x + 1*ey*math.cos(Okebot.yaw) + ey*disturbance.force_y # + inertial force
                Okebot.y = Okebot.y + 1*ex*math.sin(Okebot.yaw) + ex*disturbance.force_x # + inertial force
                case = "x"
                direction = 0

            elif diff_deg >= 2:
                if abs(diff_deg) > 180:
                    Okebot.yaw = Okebot.yaw - math.radians(2) # + inertial force
                    case = "r"
                    direction = 1
                else:
                    Okebot.yaw = Okebot.yaw + math.radians(2) # + inertial force
                    case = "r"
                    direction = 0
            elif diff_deg < -2:
                if abs(diff_deg) > 180:
                    Okebot.yaw = Okebot.yaw + math.radians(2) # + inertial force
                    case = "r"
                    direction = 0
                else:
                    Okebot.yaw = Okebot.yaw - math.radians(2) # + inertial force
                    case = "r"
                    direction = 0
                #print("my deg: ", math.degrees(Okebot.yaw))
            disturbance.shift_wave_term()
            disturbance.shift_window_term()
            disturbance.change_disturbance_force()
            csvWriter.writerow([target_point[0], target_point[1],
                                current_point[0], current_point[1],
                                current_yaw])
            time.sleep(0.05)
            T = therust(case, direction)
            #print(T)
    print("Mission complete")
    file.close()
except KeyboardInterrupt:
    file.close()