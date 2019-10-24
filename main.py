# -*- coding: utf-8 -*-
"""
Created on Thu Oct 24 14:43:13 2019

@author: FujiiChang
"""

import csv

file = open('status.csv', 'a')
csvWriter = csv.writer(file) 
csvWriter.writerow(['longitude', 'latitude', 'yaw'])

def save_state_data(lon, lat, yaw):
    csvWriter.wirterow([lon,  lat, yaw])

def draw_state(lon, lat, yaw):
    pass

lon = 135.00
lat = 35.00
yaw = 0.0

save_status_data(lon, lat, yaw)
draw_status(lon, lat, yaw)


