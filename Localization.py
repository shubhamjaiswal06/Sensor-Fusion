# -*- coding: utf-8 -*-
"""
Created on Sat Dec 26 10:22:32 2020

@author: AB
"""
#import os
import numpy as np
import pandas as pd
#import csv
# a= np.array(['Hallo','World'])
# a= np.append(a,'@')
# print("current array : {}".format(a))

# for i in a:
#     print(i)
prev_time = None
prev_QR = None
QR_codes = set()
file_path = "E:/ashutosh/sensorfusionstuff/shubham project/dataset3/data/task5"
file_name = "camera_localization_task5.csv"
f_length = 524 #focal length in pixels
#data = csv.reader(file_path+file_name,delimiter=',')
data = np.genfromtxt(file_path+'/'+file_name,delimiter=',')
QR_codes.update(data[:,1])
QR_codes =sorted(QR_codes)
for row in data:
    curr_time = row[0]
    print(curr_time)
    if prev_time == None:
        prev_time = curr_time
        count = 0
    if prev_time == curr_time:
        count += 1
    else:
        print(count)
        break
        
sorted_data = np.sort(data,axis=0)
TrueDistance = sorted_data[:,6]+6.7

for row in sorted_data:
    curr_QR = row[1]
    
    if prev_QR == None:
        prev_QR = curr_QR
        count = 0
    if prev_QR == curr_QR:
        
        count += 1
        print(curr_QR)
    else:
        print(count)
        break