# -*- coding: utf-8 -*-
"""
Created on Sat Dec 26 10:22:32 2020

@author: AB
"""
#import os
import numpy as np
import math , mpmath
import matplotlib.pyplot as plt
import time

def initilization():
    # Initial assumption about global position and attitude of robot
    global robot_estimated_pose,psyi,Measurement,actual_height,f_length,step_size
    psyi = 50
    psyi = np.radians(psyi)
    
    robot_estimated_pose = np.array([[20], [40]])

    Measurement = np.zeros((7,1))           # considered initial measurement 
    
    actual_height = 11.5                        # Actual height in cm

    f_length = 524                              #focal length in pixels
    step_size = 0.002                            # Gradient Decent Step_size


def estimate(QR_global_pose,robot_estimated_pose):
    
    g_x = np.zeros((7,1))
    
    g_x[:,0] = np.array(actual_height * f_length /(np.sqrt(pow(QR_global_pose[:,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[:,1] - robot_estimated_pose[1,0],2)))) 

    return g_x
    

def Gradient_decent_cost_function(QR_global_pose,robot_estimated_pose,Measurement,R_Matrix):
      
    g_x = estimate(QR_global_pose,robot_estimated_pose)
    Jacobean_Mat = jacobean_calculations(actual_height, f_length,QR_global_pose,robot_estimated_pose)
    
    #R-inverse
    R_Inverse = np.linalg.inv(R_Matrix)
    M = (Measurement - g_x)
    #Gradient_decent_cost = Jacobean_Mat.transpose() * (Measurement - Estimated)
    #Z = np.matmul(Jacobean_Mat.transpose(),R_Inverse)
    Gradient_decent_cost =  np.matmul(Jacobean_Mat.transpose(),M)
    return Gradient_decent_cost

def jacobean_calculations(actual_height, f_length,QR_global_pose,robot_estimated_pose):
    
    ##This function creates the jacobean matrix for calculations ##
    # For the current measurement model, 2 rows and 3 colomns jacobean will be created
    Jacobean_Mat = np.zeros((7,2))  # Initial Jacobean Matrix
    
    Jacobean_Mat[:,0] = np.array(-actual_height * f_length * (QR_global_pose[:,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[:,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[:,1] - robot_estimated_pose[1,0],2),3/2)))
    Jacobean_Mat[:,1] = np.array(-actual_height * f_length * (QR_global_pose[:,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[:,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[:,1] - robot_estimated_pose[1,0],2),3/2)))
      
    return Jacobean_Mat


t0 = time.time()
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
avg_center_pos_y = []
var_center_pos = []

for row in sorted_data:
    curr_QR = row[1]
    
    if prev_QR == None:
        prev_QR = curr_QR
        #Create height and center pos buffers
        measured_height = []
        center_pos = []
        center_pos_y = []
    if prev_QR == curr_QR:
        # Fill the respective buffer
        measured_height.append(row[5])
        center_pos.append(row[2])
        center_pos_y.append(row[3])
    else:
        #Measure the variance in measurement from the buffers
        var_measured_height.append(np.var(measured_height))
        var_center_pos.append(np.var(center_pos))
        avg_measured_height.append(np.mean(measured_height))
        avg_center_pos.append(np.mean(center_pos))     
        avg_center_pos_y.append(np.mean(center_pos_y))
        #clear the buffers
        measured_height.clear()
        center_pos.clear()
        center_pos_y.clear()
        prev_QR = curr_QR
        # add the height and center pos into the buffer
        measured_height.append(row[5])
        center_pos.append(row[2])
        center_pos_y.append(row[3])
    
#Measure the last variance of measurement from buffer
var_measured_height.append(np.var(measured_height))
var_center_pos.append(np.var(center_pos))
avg_measured_height.append(np.mean(measured_height))
avg_center_pos.append(np.mean(center_pos))
avg_center_pos_y.append(np.mean(center_pos_y))
var_R = np.array(var_center_pos)

#read the global QR position file
file_path = "E:/ashutosh/sensorfusionstuff/shubham project"
file_name = "qr_code_position_in_global_coordinate.csv"
global_pos_Data = np.genfromtxt(file_path+'/'+file_name,delimiter=',')
global_pos_Data = global_pos_Data[1:,:]

global_Xpos_QR = [25,37,50.5,62.5,74.5,86.5,98]
global_Ypos_QR = [121.5,121.5,121.5,121.5,121.5,121.5,121.5]

prev_QR = None      # re-initilized for next loop
QR_Codes_with_var = np.zeros((7,3))

#QR_Codes_with_var[:,0] = QR_codes
#QR_Codes_with_var[:,1] = var_center_pos
#QR_Codes_with_var[:,2] = var_measured_height
#QR_code_index = np.where(global_pos_Data == curr_QR)
#QR_global_pose = [global_pos_Data[QR_code_index[0][0],1],global_pos_Data[QR_code_index[0][0],2]]
initilization()
pose_x = []
pose_y = []
attitude = []
num_iterations = 10000

var_measured_height[3] = np.min(np.ma.masked_equal(var_measured_height,0,copy=False))/2
var_center_pos[3] =  np.min(np.ma.masked_equal(var_center_pos,0,copy=False))/2
Total_data = np.zeros((7,7))
Total_data[:,0] = QR_codes
Total_data[:,1] = avg_measured_height
Total_data[:,2] = avg_center_pos
Total_data[:,3] = global_Xpos_QR
Total_data[:,4] = global_Ypos_QR
Total_data[:,5] = var_measured_height
Total_data[:,6] = var_center_pos       

alpha = []
# global_Cx_pos = []
# for i in range(7):
#     a = math.atan2(f_length,avg_center_pos[i])
#     alpha.append(a)
#     global_Cx_pos.append(np.cos(a)*avg_center_pos[i] + np.sin(a)*avg_center_pos_y[i])
    
QR_global_pose = np.array([global_Xpos_QR, global_Ypos_QR]).transpose() 
m= avg_measured_height
Measurement[:,0] = m
R_Matrix = np.diag(var_measured_height+var_center_pos)

for i in range(num_iterations):
    update_direction = 0 
    update_direction += Gradient_decent_cost_function(QR_global_pose,robot_estimated_pose,Measurement,R_Matrix)
    robot_estimated_pose = np.subtract(robot_estimated_pose , step_size * update_direction)
    
    pose_x.append(robot_estimated_pose[0])
    pose_y.append(robot_estimated_pose[1])
    #attitude.append(robot_estimated_pose[2])

plt.plot(pose_x,label="P_x")
plt.plot(pose_y,label="P_y")
plt.ylabel("Position values")
plt.xlabel("iterations")
plt.legend()
plt.show()
print(time.time() - t0)
