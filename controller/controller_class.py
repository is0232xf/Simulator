# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 14:53:49 2020

@author: FujiiChang
"""
import math
import time
import numpy as np
import calculate_angle
import calculate_degree as cal_deg
from geopy.distance import geodesic

class Controller:
    def __init__(self, way_point):
        self.way_point = way_point
        self.way_point_num = 0
        self.next_goal = self.way_point[self.way_point_num]

    def check_way_point(self):
        if self.way_point_num > len(self.way_point)-2:
            return True
        else:
            return False

    def update_way_point(self):
        self.way_point_num = self.way_point_num + 1
        self.next_goal = self.way_point[self.way_point_num]

    def decide_next_action(self, Okebot):
        # decide the next action from current robot status and the next waypoint
        current_point = np.array([Okebot.x, Okebot.y])
        current_yaw = Okebot.yaw
        diff_distance = geodesic(current_point, self.next_goal).m
        print(diff_distance)
        # check distance between current and target
        if abs(diff_distance) < 0.5:
            print("complete mission ", self.way_point_num)
            action = ["s", 0]
            self.update_way_point()
            print("change waypoint")
            print("next way point: ", self.next_goal)
            time.sleep(1)
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
            print("next way point: ", self.next_goal)
        print("action: ", action)
        return action

    def update_pwm_pulse(self, action):
        # update pwm pulse width for each thruster from the next action
        T1 = 1500
        T2 = 1500
        T3 = 1500
        T4 = 1500


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