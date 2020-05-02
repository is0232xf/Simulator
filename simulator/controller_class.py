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
        params_file = open("params.json", "r")
        params = json.load(params_file) 
        self.tolerance = params["Controller"]["torelance"]
        self.way_point = way_point
        self.way_point_num = 0
        self.next_goal = self.way_point[self.way_point_num]

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
        if action[0] == "s" and action[1] == 0:
            T1 = 1500
            T2 = 1500
            T3 = 1500
            T4 = 1500
        elif action[0] == "x" and action[1] == 0:
            T1 = 1600
            T2 = 1600
            T3 = 1400
            T4 = 1400
        elif action[0] == "x" and action[1] == 1:
            T1 = 1400
            T2 = 1400
            T3 = 1600
            T4 = 1600
        elif action[0] == "y" and action[1] == 0:
            T1 = 1600
            T2 = 1400
            T3 = 1600
            T4 = 1400
        elif action[0] == "y" and action[1] == 1:
            T1 = 1400
            T2 = 1600
            T3 = 1400
            T4 = 1600
        elif action[0] == "r" and action[1] == 0:
            T1 = 1600
            T2 = 1400
            T3 = 1400
            T4 = 1600
        elif action[0] == "r" and action[1] == 1:
            T1 = 1400
            T2 = 1600
            T3 = 1600
            T4 = 1400

        T = np.array([[T1],
                      [T2],
                      [T3],
                      [T4]])
        return T
