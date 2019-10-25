# -*- coding: utf-8 -*-
"""
Created on Fri Oct 25 16:54:24 2019

@author: FujiiChang
"""

import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("udpin:0.0.0.0:14551")
print("Succeeded to connection")

msg = None
while msg is None:
    the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
    print(msg)
    time.sleep(1)
print("Finish")
