# -*- coding: utf-8 -*-
"""
Created on Thu Nov 28 12:58:03 2019

@author: FujiiChang
"""

import csv
import numpy as np

filename = "star.csv"
file = open(filename, 'a', newline='')
csvWriter = csv.writer(file)

way_point = np.array([[34.982202, 135.963644],
                      [34.982111, 135.963602],
                      [34.982166, 135.963707],
                      [34.982170, 135.963586],
                      [34.982109, 135.963690],
                      [34.982202, 135.963644]])

for data in way_point:
    csvWriter.writerow(data)

file.close()