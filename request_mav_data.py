# -*- coding: utf-8 -*-
"""
Created on Fri Oct 25 16:54:24 2019

@author: FujiiChang
"""

from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)