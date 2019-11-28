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
file = open('../../csv_data/'+ date +'.csv', 'a', newline='')
csvWriter = csv.writer(file)
# set up vehicle
print("waiting connection")

way_point = np.array([[34.982187, 135.963695], [34.982192, 135.963596], [34.982118, 135.963598],
                      [34.982118, 135.963698]])
"""
way_point = np.array([[34.980779, 135.963660], [34.981122, 135.963526], [34.981425, 135.963706], [34.981598, 135.963561],
                      [34.981598, 135.963561], [34.981095, 135.963695],[34.980350, 135.963502]])
"""
print(len(way_point))
way_point_num = 0
target_point = way_point[way_point_num]

my_position = np.array([34.982118, 135.963698, math.radians(90)])
try:
    while True:
        current_point = np.array([my_position[0], my_position[1]])
        # current_yaw = BIWAKO.vehicle.heading
        current_yaw = my_position[2]
        diff_distance = geodesic(current_point, target_point).m
        
        # check distance between current and target
        if abs(diff_distance) < 1:
            print("complete mission ", way_point_num)
            time.sleep(2)
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
            print("target deg: ", target_direction)
            current_yaw = math.degrees(current_yaw)
            diff_deg =  math.degrees(calculate_angle.limit_angle(math.radians(target_direction - current_yaw)))
            print("diff deg: ", diff_deg)
            # when the device aims to the target point
            if abs(diff_deg) < 2:
                my_position[0] = my_position[0] + 2*ey*math.cos(my_position[2])
                my_position[1] = my_position[1] + 2*ex*math.sin(my_position[2])
                print("!!!")
    
            elif diff_deg >= 2:
                if abs(diff_deg) > 180:                  
                    my_position[2] = my_position[2] - math.radians(2)
                else:
                    my_position[2] = my_position[2] + math.radians(2)
                print("my deg: ", math.degrees(my_position[2]))
                
            elif diff_deg < -2:
                if abs(diff_deg) > 180:                  
                    my_position[2] = my_position[2] + math.radians(2)
                else:
                    my_position[2] = my_position[2] - math.radians(2)
                print("my deg: ", math.degrees(my_position[2]))
            csvWriter.writerow([target_point[0], target_point[1], 
                                current_point[0], current_point[1], 
                                current_yaw])
            time.sleep(0.2)
    print("Mission complete")
    file.close()
except KeyboardInterrupt:
    file.close()