# -*- coding: utf-8 -*-
"""
Created on Sat Dec 26 10:22:32 2020

@author: AB
"""
#import os
import numpy as np

#import csv
# a= np.array(['Hallo','World'])
# a= np.append(a,'@')
# print("current array : {}".format(a))

# for i in a:
#     print(i)


def Gradient_decent_cost_function(actual_height,f_length,QR_global_pose,robot_estimated_pose,psyi,Measurement):
    
    first_term = QR_global_pose[0] - robot_estimated_pose[0]
    second_term = QR_global_pose[0] - robot_estimated_pose[0]
    Square_both = first_term**2 + second_term**2
    
    estimated_height_QR = actual_height * f_length /(np.sqrt(Square_both)) 
    estimated_Cx = f_length * np.tan(np.arctan(second_term/first_term) - psyi)
    Estimated = np.array([[estimated_height_QR], [estimated_Cx]])
    
    Jacobean_Mat = jacobean_calculations(actual_height, f_length,first_term,second_term,Square_both,psyi)
    
    Gradient_decent_cost = Jacobean_Mat.transpose() * (Measurement - Estimated)
    
    return Gradient_decent_cost

def jacobean_calculations(actual_height, f_length,first_term,second_term,Square_both,psyi):
    
    ##This function creates the jacobean matrix for calculations ##
    # For the current measurement model, 2 rows and 3 colomns jacobean will be created
    
    Jacobean_Mat[0,0] = actual_height * f_length * (first_term) / pow(Square_both,3/2)
    Jacobean_Mat[0,1] = actual_height * f_length * (second_term) / pow(Square_both,3/2)
    Jacobean_Mat[0,2] = 0
    Jacobean_Mat[1,0] =  f_length * pow(np.arccos(np.arctan(second_term/first_term) - psyi),2) * (second_term / Square_both)
    Jacobean_Mat[1,1] = -f_length * pow(np.arccos(np.arctan(second_term/first_term) - psyi),2) * (first_term / Square_both)
    Jacobean_Mat[1,2] = -f_length * pow(np.arccos(np.arctan(second_term/first_term) - psyi),2) 
    
    return Jacobean_Mat


prev_time = None
prev_QR = None
QR_codes = set()
file_path = "E:/ashutosh/sensorfusionstuff/shubham project/dataset3/data/task5"
file_name = "camera_localization_task5.csv"

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
        
sorted_data = sorted(data,key=lambda x: x[1])
#sorted_data[:,6] = sorted_data[:,6]+6.7
avg_measured_height = []
var_measured_height = []
avg_center_pos = []
var_center_pos = []
P_x = 35
P_y = 46
Sigma = 75
Measurement = np.array([[0],[2]])           # considered initial measurement 
Jacobean_Mat = np.array([[0,0,0],[0,0,0]])  # Initial Jacobean Matrix
actual_height = 11.5                        # Actual height in cm

f_length = 524                              #focal length in pixels
step_size = 0.02                            # Gradient Decent Step_size


for row in sorted_data:
    curr_QR = row[1]
    
    if prev_QR == None:
        prev_QR = curr_QR
        measured_height = []
        center_pos = []
    if prev_QR == curr_QR:
        measured_height.append(row[5])
        center_pos.append(row[2])
    else:
        avg_measured_height.append(np.mean(measured_height))
        var_measured_height.append(np.var(measured_height))
        
        var_center_pos.append(np.var(center_pos))
        print(np.mean(measured_height))
        print(np.var(measured_height))
        measured_height.clear()
        center_pos.clear()
        prev_QR = curr_QR
        measured_height.append(row[5])
        center_pos.append(row[2])
        
avg_measured_height.append(np.mean(measured_height))
var_measured_height.append(np.var(measured_height))

var_center_pos.append(np.var(center_pos))

