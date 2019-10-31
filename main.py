# -*- coding: utf-8 -*-
"""
Created on Thu Oct 24 14:43:13 2019

@author: FujiiChang
"""

import os
import csv
import time
import math
import datetime
import matplotlib.pyplot as plt
import numpy as np
from dronekit import connect

the_connection = connect("udpin:0.0.0.0:14551", wait_ready=True)
print("Succeeded to connection")

theta = 34.9820933 # 緯度
earth_R = 6378137 # 地球の半径．WGS84の値を用いる
ey = 360/(2*math.pi*earth_R) # 緯度単位距離. 1mあたりの度数
ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # 経度単位距離. 1mあたりの度数

# プログtラム開始時の日時を保持
detail = datetime.datetime.now()
date = detail.strftime("%Y_%m_%d_%H_%M_%S")
# 各フレームの画像を格納するフォルダを作成
os.makedirs("plot_data/" + date)

file = open('./csv/'+ date +'.csv', 'a')
csvWriter = csv.writer(file)
csvWriter.writerow(['heading', 'longitude', 'latitude', 'CH5', 'CH6'])

def save_state_data(file, heading, lon, lat, CH5, CH6):
    csvWriter.writerow([heading, lon, lat, CH5, CH6])

def calc_vector(current, channels):
    c_lon = current[0]
    c_lat = current[1]
    forward_point = np.array([c_lon, c_lat+channels[1]])
    lateral_point = np.array([c_lon+channels[0], c_lat])
    synthetic_point = forward_point + lateral_point - current
    
    R = np.array([[math.cos(theta), -math.sin(theta)],
                   [math.sin(theta), math.cos(theta)]])
    T = np.array([[synthetic_point[0]-c_lon],
                  [synthetic_point[1]-c_lat]])
    C = np.array([[c_lon], 
                  [c_lat]])
    
    synthtic_point = R @ T + C
    return synthtic_point

def draw_state(theta, current, target, channels, data_num):
    name = (str(data_num) + ".jpg")
    c_lon = current[0]
    c_lat = current[1]
    t_lon = target[0]
    t_lat = target[1]
    synthtic_point = calc_vector(current, channels)
    plt.xlabel("longitude")
    plt.ylabel("latitude")
    plt.xlim([t_lon-18*ex, t_lon+18*ex])
    plt.ylim([t_lat-12*ey, t_lat+12*ey])
    plt.scatter([c_lon], [c_lat])
    plt.scatter([t_lon], [t_lat])
    plt.arrow(c_lon, c_lat, synthtic_point[0][0]-c_lon, synthtic_point[1][0]-c_lat, width=0.000001, head_width=0.000005, fc="green")
    plt.show()
    plt.savefig("plot_data/" + str(date) + "/" + name)
    plt.clf()
    
if __name__ == "__main__":
    while True:
        point = the_connection.location.global_frame
        print(point)
        current_point = np.array([34.982114, 135.963686])
        target_point = np.array([34.982168, 135.963615])
        heading = the_connection.attitude.yaw
        
        # input channel value 
        CH5 = the_connection.channels['5']
        CH6 = the_connection.channels['6']
        
        channels = np.array([(CH5-1500)*ex/100, (CH6-1500)*ey/100])
    
        current = np.array([current_point[1], current_point[0]])
        target = np.array([target_point[1], target_point[0]])
        
        save_state_data(file, heading, current[0], current[1], CH5, CH6)
        # draw_state(theta, current, target, channels) 
    
    file.close()
