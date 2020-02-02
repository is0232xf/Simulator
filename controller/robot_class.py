# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:15:26 2019

@author: FujiiChang
"""

import math
import matplotlib.patches as patches

R = 6378137 # Earth radius WGS84

class Robot:
    def __init__(self, pose, color="black"):
        self.pose = pose
        self.x = self.pose[0]
        self.y = self.pose[1]
        self.yaw = self.pose[2]
        self.r = 0.000005
        self.color = color
        
    def move(self, power, omega):
        """
        ey = 360/(2*math.pi*R) # latitude: 1deg -> 1m
        ex = 360/(2*math.pi*R*math.cos(self.pose[0]*math.pi/180)) # longitude: 1deg -> 1m
        """
        t_x =  math.cos(self.pose[2])*power
        t_y = math.sin(self.pose[2])*power
        theta = math.radians(omega)
        print("omega: ", omega)
        print("theta: ", theta)
        
        print("current pose: ", self.pose)
        self.pose[0] = self.pose[0] + t_x
        self.pose[1] = self.pose[1] + t_y
        self.pose[2] = self.pose[2] + theta
        print("move to: ", self.pose)
 
    def therust(case, direction):
        # 入力をx方向への直進，y方向への直進，回転にする (str型)
        # 出力はスラスト力をndarray型にする
        
        # T is a list that has each motor 
        T = np.array([[0],
                      [0],
                      [0],
                      [0]])
        if case == "x":
            if direction == 0:
                T1.set_pwm_input(1600)
                T2.set_pwm_input(1600)
                T3.set_pwm_input(1400)
                T4.set_pwm_input(1400)
                
            elif direction == 1:
                T1.set_pwm_input(1400)
                T2.set_pwm_input(1400)
                T3.set_pwm_input(1600)
                T4.set_pwm_input(1600) 
            
            
        elif case == "y":
            if direction == 0:
                T1.set_pwm_input(1600)
                T2.set_pwm_input(1400)
                T3.set_pwm_input(1600)
                T4.set_pwm_input(1400)
                
                
            elif direction == 1:
                T1.set_pwm_input(1400)
                T2.set_pwm_input(1600)
                T3.set_pwm_input(1400)
                T4.set_pwm_input(1600) 
            
        elif case == "r":
            if direction == 0:
                T1.set_pwm_input(1600)
                T2.set_pwm_input(1400)
                T3.set_pwm_input(1400)
                T4.set_pwm_input(1600)
                
                
            elif direction == 1:
                T1.set_pwm_input(1400)
                T2.set_pwm_input(1600)
                T3.set_pwm_input(1600)
                T4.set_pwm_input(1400)
    
        thrust_1 = T1.determine_thrust()
        thrust_2 = T2.determine_thrust()
        thrust_3 = T3.determine_thrust()
        thrust_4 = T4.determine_thrust()
        
        T = np.array([[thrust_1],
                      [thrust_2],
                      [thrust_3],
                      [thrust_4]])
    
        a = math.cos(math.radians(45))
        b = math.sin(math.radians(45))
        D = 0.25 # unit[m]
        phi = np.array([[a, a, -a, -a],
                        [b, -b, b, -b],
                        [D, -D, -D, D]])
        t = np.dot(phi, T)
        print(t)
        return t
