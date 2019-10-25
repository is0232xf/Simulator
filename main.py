# -*- coding: utf-8 -*-
"""
Created on Thu Oct 24 14:43:13 2019

@author: FujiiChang
"""

#import cv2
import csv
import math
import matplotlib.pyplot as plt

theta = 34.9820933 # 緯度
earth_R = 6378137 # 地球の半径．WGS84の値を用いる
ey = 360/(2*math.pi*earth_R) # 緯度単位距離. 1mあたりの度数
ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # 経度単位距離. 1mあたりの度数

file = open('state.csv', 'a')
csvWriter = csv.writer(file)
csvWriter.writerow(['longitude', 'latitude', 'yaw'])

def save_state_data(file, lon, lat, yaw):
    csvWriter.writerow([lon, lat, yaw])

def draw_state(current, target):
    c_lon = current[0]
    c_lat = current[1]
    t_lon = target[0]
    t_lat = target[1]
    plt.xlim([t_lon-12*ex, t_lon+12*ex])
    plt.ylim([t_lat-8*ey, t_lat+8*ey])
    plt.scatter([c_lon], [c_lat])
    plt.scatter([t_lon], [t_lat])
    plt.arrow(c_lon, c_lat, t_lon-c_lon, t_lat-c_lat, width=0.000001, head_width=0.000005, fc="red")
    plt.show()
    
if __name__ == "__main__":
    
    current_point = [34.982114, 135.963686, 0.0]
    target_point = [34.982168, 135.963615]

    current = [current_point[1], current_point[0], current_point[2]]
    target = [target_point[1], target_point[0]]
    
    save_state_data(file, current[0], current[1], current[2])
    draw_state(current, target)
    
    file.close()
