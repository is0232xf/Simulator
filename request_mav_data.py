# -*- coding: utf-8 -*-
"""
Created on Fri Oct 25 16:54:24 2019

@author: FujiiChang
"""

import time
from dronekit import connect

the_connection = connect("udpin:0.0.0.0:14551", wait_ready=True)
print("Succeeded to connection")

while True:
	print("CH3: %s" % the_connection.channels['3'])
	print("CH4: %s" % the_connection.channels['4'])
	print("CH5: %s" % the_connection.channels['5'])
	print("CH6: %s" % the_connection.channels['6'])
	time.sleep(1)

print("Finish")
