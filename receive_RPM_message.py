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
    rpm = the_connection.messages['ESC_TELEMETRY_1_TO_4']
    print(rpm)
    time.sleep(0.5)
