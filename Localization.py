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


def cost_function(Jacobean_Mat,measurement,estimated):
    print("Inside cost function")
    cost = Jacobean_Mat.transpose() * (measurement - estimated)
    return cost

def jacobean_calculations(actual_height, f_length,QR_global_pose,robot_estimated_pose,psyi):
    
    ##This function creates the jacobean matrix for calculations ##
    # For the current measurement model, 2 X 3 jacobean will be created
    
    first_term = QR_global_pose[0] - robot_estimated_pose[0]
    second_term = QR_global_pose[0] - robot_estimated_pose[0]
    Square_both = first_term**2 + second_term**2
    
    Jacobean_Mat[0,0] = actual_height * f_length * (first_term) / pow(Square_both,3/2)
    Jacobean_Mat[0,1] = actual_height * f_length * (second_term) / pow(Square_both,3/2)
    Jacobean_Mat[0,2] = 0
    Jacobean_Mat[1,0] = f_length  * pow(np.arccos(np.arctan(second_term/first_term) - psyi),2) * (second_term / Square_both)
    Jacobean_Mat[1,1] = -f_length * pow(np.arccos(np.arctan(second_term/first_term) - psyi),2) * (first_term / Square_both)
    Jacobean_Mat[1,2] = -f_length * pow(np.arccos(np.arctan(second_term/first_term) - psyi),2) 
    #Jacobean_Mat = 
    
    return Jacobean_Mat
    
    
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
sorted_data[:,6] = sorted_data[:,6]+6.7
avg_distance = []
var_distance = []
P_x = 35
P_y = 46
Sigma = 75

Jacobean_Mat = np.array([[0,0,0],[0,0,0]])
for row in sorted_data:
    curr_QR = row[1]
    
    if prev_QR == None:
        prev_QR = curr_QR
        distance = []
    if prev_QR == curr_QR:
        distance.append(row[6])
        
    else:
        avg_distance.append(np.mean(distance))
        var_distance.append(np.var(distance))
        print(np.mean(distance))
        print(np.var(distance))
        distance.clear()
        prev_QR = curr_QR
        distance.append(row[6])
        
avg_distance.append(np.mean(distance))
var_distance.append(np.var(distance))


