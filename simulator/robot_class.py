# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 12:15:26 2019

@author: FujiiChang
"""

import math
import numpy as np
from earth_class import Earth
import matplotlib.patches as patches
from thruster_class import Thruster
from GPS_class import GPS

class Robot:
    def __init__(self, pose, color="black"):
        # Drawing params
        self.r = 0.000005
        self.color = color

        # Robot pose params
        self.pose = pose
        self.x = self.pose[0]
        self.y = self.pose[1]
        self.yaw = self.pose[2]
        self.voltage = 11.1

        # The distance from the center of robot to each thruster
        self.D = 0.10

        # Each thruster PWM freqs
        self.pwm_1 = 1500
        self.pwm_2 = 1500
        self.pwm_3 = 1500
        self.pwm_4 = 1500

        # Use four thrusters
        self.T1 = Thruster(self.pwm_1, self.voltage)
        self.T2 = Thruster(self.pwm_2, self.voltage)
        self.T3 = Thruster(self.pwm_3, self.voltage)
        self.T4 = Thruster(self.pwm_4, self.voltage)

        # Thrust power
        self.t1  = self.T1.calc_thrust(self.pwm_1)
        self.t2  = self.T2.calc_thrust(self.pwm_2)
        self.t3  = self.T3.calc_thrust(self.pwm_3)
        self.t4  = self.T4.calc_thrust(self.pwm_4)

        self.gps = GPS(self.x, self.y)
        self.earth = Earth(self.y)

    def update_state(self, disturbance):
        trque = self.thrust()
        yaw = self.yaw
        self.earth.calc_ex(self.earth.ey)
        R = np.array([[math.cos(yaw), -math.sin(yaw)],
                      [math.sin(yaw), math.cos(yaw)]])
        
        F = trque - disturbance
        F_t = np.array([[F[0][0]],
                        [F[1][0]]])
        F_v = np.dot(R, F_t)
        # 以下の3式の最終項の係数は現在てきとうに設定している
        F_x = self.earth.ex * F_v[0][0] * 0.15
        F_y = self.earth.ey * F_v[1][0] * 0.15
        F_r = math.radians(2) * F[2][0]
        self.x = self.x + F_x
        self.y = self.y + F_y
        self.yaw = self.yaw + F_r

        self.pose[0] = self.x
        self.pose[1] = self.y
        self.pose[2] = self.yaw

    def update_pwm_pulse(self, pwm):
        self.pwm_1 = pwm[0][0]
        self.pwm_2 = pwm[1][0]
        self.pwm_3 = pwm[2][0]
        self.pwm_4 = pwm[3][0]

    def update_thrust(self):
        self.t1 = self.T1.calc_thrust(self.pwm_1)
        self.t2 = self.T2.calc_thrust(self.pwm_2)
        self.t3 = self.T3.calc_thrust(self.pwm_3)
        self.t4 = self.T4.calc_thrust(self.pwm_4)

    def thrust(self):
        # 入力をx方向への直進，y方向への直進，回転にする (str型)
        # 出力はスラスト力をndarray型にする
        self.update_thrust()

        T = np.array([[self.t1],
                      [self.t2],
                      [self.t3],
                      [self.t4]])
        # T is a list that has each motor
        a = math.cos(math.radians(45))
        b = math.sin(math.radians(45))
        D = self.D
        phi = np.array([[a, a, -a, -a],
                        [b, -b, b, -b],
                        [D, -D, -D, D]])
        t = np.dot(phi, T)
        return t

    def draw(self, ax):
        x, y, theta = self.pose
        xn = x + self.r * math.sin(theta)
        yn = y + self.r * math.cos(theta)
        ax.plot([x, xn], [y, yn], color=self.color)
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)
        ax.add_patch(c)
