# -*- coding: utf-8 -*-
"""
Created on Thu Oct 24 14:43:13 2019

@author: FujiiChang
"""

import csv
import math
import matplotlib.pyplot as plt
import numpy as np

theta = 34.9820933 # 緯度
earth_R = 6378137 # 地球の半径．WGS84の値を用いる
ey = 360/(2*math.pi*earth_R) # 緯度単位距離. 1mあたりの度数
ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # 経度単位距離. 1mあたりの度数

file = open('state.csv', 'a')
csvWriter = csv.writer(file)
csvWriter.writerow(['longitude', 'latitude', 'yaw'])

def save_state_data(file, lon, lat, yaw):
    csvWriter.writerow([lon, lat, yaw])

def draw_state(theta, current, target, channels):
    c_lon = current[0]
    c_lat = current[1]
    t_lon = target[0]
    t_lat = target[1]

    plt.xlim([t_lon-18*ex, t_lon+18*ex])
    plt.ylim([t_lat-12*ey, t_lat+12*ey])
    plt.scatter([c_lon], [c_lat])
    plt.scatter([t_lon], [t_lat])
    
    forward_point = np.array([c_lon, c_lat+channels[1]])
    lateral_point = np.array([c_lon+channels[0], c_lat])
    synthtic_point = forward_point + lateral_point - current
    
    R = np.array([[math.cos(theta), -math.sin(theta)],
                   [math.sin(theta), math.cos(theta)]])
    T = np.array([[synthtic_point[0]-c_lon],
                  [synthtic_point[1]-c_lat]])
    C = np.array([[c_lon], 
                  [c_lat]])
    
    synthtic_point = R @ T + C
    #plt.scatter([synthtic_point[0]], [synthtic_point[1]], color="green")
    #plt.scatter([lateral_point[0]], [lateral_point[1]], color="blue")
    print(synthtic_point[1][0])
    plt.arrow(c_lon, c_lat, synthtic_point[0][0]-c_lon, synthtic_point[1][0]-c_lat, width=0.000001, head_width=0.000005, fc="green")
    #plt.arrow(c_lon, c_lat, new_point[0][0]-c_lon, new_point[1][0]-c_lat, width=0.000001, head_width=0.000005, fc="red")
    #plt.arrow(c_lon, c_lat, 0, ey*2, width=0.000004, head_width=0.000008, fc="red")
    #plt.arrow(c_lon, c_lat, ex*2, 0, width=0.000004, head_width=0.000008, fc="blue")
    plt.show()
    
if __name__ == "__main__":
    
    current_point = np.array([34.982114, 135.963686])
    target_point = np.array([34.982168, 135.963615])
    theta = math.radians(0)
    
    CH5 = 1720
    CH6 = 1690
    
    channels = np.array([(CH5-1500)*ex/100, (CH6-1500)*ey/100])

    current = np.array([current_point[1], current_point[0]])
    target = np.array([target_point[1], target_point[0]])
    
    save_state_data(file, current[0], current[1], theta)
    draw_state(theta, current, target, channels) 
    
    file.close()
