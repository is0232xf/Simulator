# -*- coding: utf-8 -*-
"""
Created on Thu Nov  7 16:54:01 2019

@author: FujiiChang
"""

import time
import math
import calculate_angle
import send_command
import numpy as np
import calculate_degree as cal_deg
from ArduPilotDriver import ArduPilotDriver
from geopy.distance import geodesic

BIWAKO = ArduPilotDriver()

BIWAKO.start()
BIWAKO.takeoff(0)

way_point = np.array([[34.5, 135.98], [34.5, 135.98], [34.5, 135.98]])
way_point_num = 0
target_point = way_point[way_point_num]

while True:
    current_point = np.array([BIWAKO.vehicle.location.global_relative_frame.lat,
                              BIWAKO.vehicle.location.global_relative_frame.lon])
    current_yaw = BIWAKO.vehicle.heading
    diff_distance = geodesic(current_point, target_point).m
    target_direction = cal_deg.calculate_bearing(current_point, target_point)
    if diff_distance < 2:
        print("Keep its position")
        time.sleep(10)
        way_point_num = way_point_num + 1
        print("Check its position again")
        if way_point_num < len(way_point):
            target_point = way_point[way_point_num]
        else:
            break
    else:
        current_yaw = math.degrees(calculate_angle.limit_angle(current_yaw))
        diff_deg = target_direction - current_yaw
        diff_deg = math.radians(diff_deg)
        diff_deg = math.degrees(calculate_angle.limit_angle(diff_deg))
        if 3 > abs(diff_deg):
            send_command.set_rc_channel_pwm(5, 1700)
        elif diff_deg >= 3:
            send_command.set_rc_channel_pwm(4, 1700)
        elif diff_deg < -3:
            send_command.set_rc_channel_pwm(4, 1300)
    time.sleep(1)
print("Mission complete")
