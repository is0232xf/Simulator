# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 16:24:06 2018

@author: FujiiChang
"""

import math
import numpy as np

class Gyrosensor(object):
    def get_gyro_x():
        pass
    def get_gyro_y():
        pass
    def get_gyro_z():
        pass

class Velocitysensor(object):
    def get_velocity():
        pass

class GPS(object):
    def get_GPS_value():
        pass

class Motor(object):
    def __init__(self, pwm_imput, voltage):
        self.pwm_imput = pwm_imput
        self.voltage = voltage

    # detrmine rotation directioin from the pwm_imput
    def determine_rotation_direction(self):
        if self.pwm_imput > 1525 and self.pwm_imput < 1900:
            return 1
        elif self.pwm_imput > 1100 and self.pwm_imput < 1475:
            return -1
        else:
            return 0
    # determine motor powew 1lbf = 4.45N
    def determine_thrust(self):
        a = 2.34375e-8
        b = -1.0078125e-4
        c = 0.152265625
        d = -80.7421875
        x = self.pwm_imput

        lbf_thrust = a*x**3 + b*x**2 + c*x + d
        if 1475 <= self.pwm_imput <= 1525:
            lbf_thrust = 0
        elif self.pwm_imput < 1100 and 1900 < self.pwm_imput:
            lbf_thrust = 0

        N_thrust = translate_lbf_to_N(lbf_thrust)
        return N_thrust

    # detemine ampere from pwm signal
    def determine_ampere(self, x):
        self.pwm_imput = x
        x = self.pwm_imput
        a = 8.4375e-5
        b = -0.253125
        c = 189.84375
        ampere = a*x**2 + b*x + c
        return ampere
    # calculate power consumption(Watt)

    def calculate_power_consumption(self):
        x = self.pwm_imput
        ampere = self.determine_ampere(x)
        power = self.voltage * ampere
        return power

def translate_lbf_to_N(lbf_thrust):
    N_thrust = 4.45 * lbf_thrust
    return N_thrust

if __name__ == "__main__":
    pwm_1 = 1400
    pwm_2 = 1600
    pwm_3 = 1600
    pwm_4 = 1400
    voltage = 11.1

    T1 = Motor(pwm_1, voltage)
    T2 = Motor(pwm_2, voltage)
    T3 = Motor(pwm_3, voltage)
    T4 = Motor(pwm_4, voltage)
    A_1 = T1.determine_ampere(pwm_1)
    thrust_1 = T1.determine_thrust()
    direction_1 = T1.determine_rotation_direction()
    power_1 = T1.calculate_power_consumption()
    A_2 = T2.determine_ampere(pwm_2)
    thrust_2 = T2.determine_thrust()
    direction_2 = T2.determine_rotation_direction()
    power_2 = T2.calculate_power_consumption()
    A_3 = T3.determine_ampere(pwm_3)
    thrust_3 = T3.determine_thrust()
    direction_3 = T3.determine_rotation_direction()
    power_3 = T3.calculate_power_consumption()
    A_4 = T4.determine_ampere(pwm_4)
    thrust_4 = T4.determine_thrust()
    direction_4 = T4.determine_rotation_direction()
    power_4 = T4.calculate_power_consumption()

    a = math.cos(math.radians(45))
    b = math.sin(math.radians(45))
    D = 0.25 # unit[m]
    phi = np.array([[a, a, -a, -a],
                    [b, -b, b, -b],
                    [D, -D, -D, D]])

    T = np.array([[thrust_1],
                  [thrust_2],
                  [thrust_3],
                  [thrust_4]])

    t = np.dot(phi, T)

    print(t)

    """
    print("Ampere: " + str(ampere) + " A")
    print("Thrust: " + str(thrust) + " N")
    print("Direction: " + str(direction) + " (plus:1, minus:-1, even:0)")
    print("Power: " + str(power) + " W")
    """