# -*- coding: utf-8 -*-
"""
Created on Thu Nov 28 12:58:03 2019

@author: FujiiChang
"""

import csv
import numpy as np

filename = "biwako.csv"
file = open(filename, 'a', newline='')
csvWriter = csv.writer(file)

way_point = np.array([[35.142080, 135.980897],
                      [35.140815, 135.979339],
                      [35.142043, 135.977730],
                      [35.143359, 135.979243],
                      [35.142080, 135.980897]])

for data in way_point:
    csvWriter.writerow(data)

file.close()