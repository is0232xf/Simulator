# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 18:40:12 2019

@author: FujiiChang
"""

import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14541')
print("Succeed to connect")

while True:
    output = the_connection.messages['SERVO_OUTPUT']
    print(output)
    time.sleep(0.5)
