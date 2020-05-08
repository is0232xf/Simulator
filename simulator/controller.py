# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 14:53:49 2020

@author: FujiiChang
"""
import math
import time
import json

import numpy as np
from geopy.distance import geodesic

import calculate_angle
import calculate_degree as cal_deg


class Controller:
    def __init__(self, way_point):
        self.params_file = open("params.json", "r")
        self.params = json.load(self.params_file) 
        self.tolerance = self.params["Controller"]["torelance"]
        self.way_point = way_point
        self.way_point_num = 0
        self.next_goal = self.way_point[self.way_point_num]

    def __del__(self):
        self.params_file.close()

    def update_way_point(self):
        self.way_point_num = self.way_point_num + 1
        self.next_goal = self.way_point[self.way_point_num]

    def decide_next_action(self, Okebot):
        action_log = False
        print_target = False
        # decide the next action from current robot status and the next waypoint
        current_point = np.array([Okebot.gps.longitude, Okebot.gps.latitude])
        current_yaw = Okebot.yaw
        diff_distance = geodesic(current_point, self.next_goal).m
        # check distance between current and target
        if abs(diff_distance) < self.tolerance:
            print("complete mission ", self.way_point_num)
            action = ["s", 0]
            if self.way_point_num < len(self.way_point)-1:
                self.update_way_point()
            else:
                action = ["f", 0]
                return action
            print("change waypoint")
            print("next way point: ", self.next_goal)
        # when the device has not received
        else:
            target_direction = math.degrees(calculate_angle.limit_angle(
                    math.radians(cal_deg.calculate_bearing(current_point, self.next_goal))))
            current_yaw = math.degrees(current_yaw)
            diff_deg =  target_direction - current_yaw
 
            if abs(diff_deg) < 2:
                action = ["x", 0]
            elif diff_deg >= 2:
                action = ["r", 0]
            elif diff_deg < -2:
                action = ["r", 1]
            if print_target:
                print("next way point: ", self.next_goal)
        if action_log:
            print("action: ", action)
        return action

    def update_pwm_pulse(self, action):
        # update pwm pulse width for each thruster from the next action
        high = 1600
        low = 3000 - high
        neutral = 1500
        if action[0] == "s" and action[1] == 0:
            T1 = neutral
            T2 = neutral
            T3 = neutral
            T4 = neutral
        elif action[0] == "x" and action[1] == 0:
            T1 = high
            T2 = high
            T3 = low
            T4 = low
        elif action[0] == "x" and action[1] == 1:
            T1 = low
            T2 = low
            T3 = high
            T4 = high
        elif action[0] == "y" and action[1] == 0:
            T1 = high
            T2 = low
            T3 = high
            T4 = low
        elif action[0] == "y" and action[1] == 1:
            T1 = low
            T2 = high
            T3 = low
            T4 = high
        elif action[0] == "r" and action[1] == 0:
            T1 = high
            T2 = low
            T3 = low
            T4 = high
        elif action[0] == "r" and action[1] == 1:
            T1 = low
            T2 = high
            T3 = high
            T4 = low

        T = np.array([[T1],
                      [T2],
                      [T3],
                      [T4]])
        return T
