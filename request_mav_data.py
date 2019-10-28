# -*- coding: utf-8 -*-
"""
Created on Fri Oct 25 16:54:24 2019

@author: FujiiChang
"""

import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("udpin:0.0.0.0:14551")
print("Succeeded to connection")

for channel in range(len(the_connection.channels)):
    str_channel = str(channel)
    print(str_channel)
    print("%s" % the_connection.channels[str_channel])
    time.sleep(1)

print("Finish")
