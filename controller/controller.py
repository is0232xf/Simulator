# -*- coding: utf-8 -*-
"""
Created on Thu Nov  7 16:54:01 2019

@author: FujiiChang
"""

import csv
import datetime
import time
import math
import calculate_angle
#import send_command
import numpy as np
import calculate_degree as cal_deg
from ArduPilotDriver import ArduPilotDriver
from geopy.distance import geodesic


theta = 34.9820933 # latitude
earth_R = 6378137 # Earth radius WGS84
ey = 360/(2*math.pi*earth_R) # latitude: 1deg -> 1m
ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # longitude: 1deg -> 1m

# get date time object
detail = datetime.datetime.now()
date = detail.strftime("%Y_%m_%d_%H_%M_%S")
# open csv file
file = open('../csv/'+ date +'.csv', 'a', newline='')
csvWriter = csv.writer(file)
# set up vehicle
print("waiting connection")
"""
BIWAKO = ArduPilotDriver()
BIWAKO.start()
BIWAKO.takeoff(0)
"""
way_point = np.array([[34.980497, 135.963574], [34.980880, 135.963589], [34.981569, 135.963623]])
print(len(way_point))
way_point_num = 0
target_point = way_point[way_point_num]

my_position = np.array([34.979778, 135.963535, 90])
while True:
    """
    current_point = np.array([BIWAKO.vehicle.location.global_relative_frame.lat,
                              BIWAKO.vehicle.location.global_relative_frame.lon])
    """
    print("position: ", my_position)
    current_point = np.array([my_position[0], my_position[1]])
    # current_yaw = BIWAKO.vehicle.heading
    current_yaw = my_position[2]
    current_yaw = math.radians(current_yaw)
    #print("current yaw: ", current_yaw)
    diff_distance = geodesic(current_point, target_point).m
    #print("distance: ", diff_distance)
    
    # check distance between current and target
    if abs(diff_distance) < 5:
        print("complete mission ", way_point_num)
        time.sleep(2)
        way_point_num = way_point_num + 1
        if way_point_num >= len(way_point):
            break
        target_point = way_point[way_point_num]
        print("change waypoint")
        print("next way point: ", target_point)
    else:
        target_direction = cal_deg.calculate_bearing(current_point, target_point)
        #print("target direction: ", target_direction)
        current_yaw = math.degrees(calculate_angle.limit_angle(current_yaw))
        #print("yaw: ", current_yaw)
        diff_deg = target_direction - current_yaw
        diff_deg = math.radians(diff_deg)
        diff_deg = math.degrees(calculate_angle.limit_angle(diff_deg))
        print("diff deg: ", diff_deg)
        print("direction: ", my_position[2])
        if 1 > abs(diff_deg):
            #send_command.set_rc_channel_pwm(5, 1700)
            #print("command: straight")
            my_position[0] = my_position[0] + 5*ey*math.sin(my_position[2])
            my_position[1] = my_position[1] - 5*ex*math.cos(my_position[2])
            print("!!!")

        elif diff_deg >= 1:
            #send_command.set_rc_channel_pwm(3, 1300)
            #print("command: turn Right")
            my_position[2] = my_position[2] + 3
        elif diff_deg < -1:
            #send_command.set_rc_channel_pwm(3, 1700)
            #print("command: turn Left")
            my_position[2] = my_position[2] - 3
        """
        csvWriter.writerow([target_point[0], target_point[1], 
                            current_point[0], current_point[1], 
                            current_yaw, 
                            BIWAKO.vehicle.channels['3'], 
                            BIWAKO.vehicle.channels['4'], 
                            BIWAKO.vehicle.channels['5'], 
                            BIWAKO.vehicle.channels['6']])
        """
        csvWriter.writerow([target_point[0], target_point[1], 
                            current_point[0], current_point[1], 
                            current_yaw])
        time.sleep(0.2)
print("Mission complete")
