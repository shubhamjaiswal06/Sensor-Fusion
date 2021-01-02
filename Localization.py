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
    psyi = 75
    psyi = np.radians(psyi)
    
    robot_estimated_pose = np.array([[0], [0],[ psyi]])
    
    
    Measurement = np.zeros((14,1))           # considered initial measurement 
    
    actual_height = 11.5                        # Actual height in cm

    f_length = 524                              #focal length in pixels
    step_size = 0.002                            # Gradient Decent Step_size


def estimate(QR_global_pose,robot_estimated_pose):
    
    g_x = np.zeros((14,1))
    
    g_x[0] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[0,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[0,1] - robot_estimated_pose[1,0],2))) 
    g_x[1] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[1,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[1,1] - robot_estimated_pose[1,0],2))) 
    g_x[2] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[2,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[2,1] - robot_estimated_pose[1,0],2))) 
    g_x[3] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[3,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[3,1] - robot_estimated_pose[1,0],2)))
    g_x[4] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[4,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[4,1] - robot_estimated_pose[1,0],2))) 
    g_x[5] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[5,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[5,1] - robot_estimated_pose[1,0],2))) 
    g_x[6] = actual_height * f_length /(np.sqrt(pow(QR_global_pose[6,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[6,1] - robot_estimated_pose[1,0],2)))
    
    g_x[7] = f_length *  math.tan(math.atan2((QR_global_pose[0,1] - robot_estimated_pose[1,0]),(QR_global_pose[0,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    g_x[8] = f_length *  math.tan(math.atan2((QR_global_pose[1,1] - robot_estimated_pose[1,0]),(QR_global_pose[1,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    g_x[9] = f_length *  math.tan(math.atan2((QR_global_pose[2,1] - robot_estimated_pose[1,0]),(QR_global_pose[2,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    g_x[10] = f_length *  math.tan(math.atan2((QR_global_pose[3,1] - robot_estimated_pose[1,0]),(QR_global_pose[3,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    g_x[11] = f_length *  math.tan(math.atan2((QR_global_pose[4,1] - robot_estimated_pose[1,0]),(QR_global_pose[4,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    g_x[12] = f_length *  math.tan(math.atan2((QR_global_pose[5,1] - robot_estimated_pose[1,0]),(QR_global_pose[5,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    g_x[13] = f_length *  math.tan(math.atan2((QR_global_pose[6,1] - robot_estimated_pose[1,0]),(QR_global_pose[6,0] - robot_estimated_pose[0,0])) - robot_estimated_pose[2,0])
    
    return g_x
    

def Gradient_decent_cost_function(QR_global_pose,robot_estimated_pose,Measurement,R_Matrix):
    
    # first_term = QR_global_pose[0] - robot_estimated_pose[0,0]
    # second_term = QR_global_pose[1] - robot_estimated_pose[1,0]
    # Square_both = first_term**2 + second_term**2
    
    # estimated_height_QR = actual_height * f_length /(np.sqrt(Square_both)) 
    # estimated_Cx = f_length * math.tan(math.atan2(second_term,first_term) - robot_estimated_pose[2,0])
    # Estimated = np.array([[0],[0]])
    # Estimated[0,0] = estimated_height_QR
    # Estimated[1,0] = estimated_Cx
     
    g_x = estimate(QR_global_pose,robot_estimated_pose)
    Jacobean_Mat = jacobean_calculations(actual_height, f_length,QR_global_pose,robot_estimated_pose)
    
    #Gradient_decent_cost = Jacobean_Mat.transpose() * (Measurement - Estimated)
    Gradient_decent_cost =  np.matmul(Jacobean_Mat.transpose(),  (Measurement - g_x))
    return Gradient_decent_cost

def jacobean_calculations(actual_height, f_length,QR_global_pose,robot_estimated_pose):
    
    ##This function creates the jacobean matrix for calculations ##
    # For the current measurement model, 2 rows and 3 colomns jacobean will be created
    Jacobean_Mat = np.zeros((14,3))  # Initial Jacobean Matrix
    
    Jacobean_Mat[0,0] = -actual_height * f_length * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[0,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[0,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[0,1] = -actual_height * f_length * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[0,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[0,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[0,2] = 0
    
    Jacobean_Mat[1,0] = -actual_height * f_length * (QR_global_pose[1,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[1,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[1,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[1,1] = -actual_height * f_length * (QR_global_pose[1,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[1,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[1,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[1,2] = 0
    
    Jacobean_Mat[2,0] = -actual_height * f_length * (QR_global_pose[2,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[2,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[2,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[2,1] = -actual_height * f_length * (QR_global_pose[2,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[2,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[2,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[2,2] = 0
    
    Jacobean_Mat[3,0] = -actual_height * f_length * (QR_global_pose[3,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[3,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[3,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[3,1] = -actual_height * f_length * (QR_global_pose[3,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[3,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[3,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[3,2] = 0
    
    Jacobean_Mat[4,0] = -actual_height * f_length * (QR_global_pose[4,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[4,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[4,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[4,1] = -actual_height * f_length * (QR_global_pose[4,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[4,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[4,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[4,2] = 0
    
    Jacobean_Mat[5,0] = -actual_height * f_length * (QR_global_pose[5,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[5,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[5,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[5,1] = -actual_height * f_length * (QR_global_pose[5,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[5,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[5,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[5,2] = 0
    
    Jacobean_Mat[6,0] = -actual_height * f_length * (QR_global_pose[6,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[6,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[6,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[6,1] = -actual_height * f_length * (QR_global_pose[6,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[6,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[6,1] - robot_estimated_pose[1,0],2),3/2))
    Jacobean_Mat[6,2] = 0
    
    Jacobean_Mat[7,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[0,1] - robot_estimated_pose[1,0])/(QR_global_pose[0,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[0,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[0,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[7,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[0,1] - robot_estimated_pose[1,0])/(QR_global_pose[0,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[0,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[0,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[7,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[0,1] - robot_estimated_pose[1,0])/(QR_global_pose[0,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    Jacobean_Mat[8,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[1,1] - robot_estimated_pose[1,0])/(QR_global_pose[1,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[1,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[1,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[8,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[1,1] - robot_estimated_pose[1,0])/(QR_global_pose[1,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[1,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[1,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[8,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[1,1] - robot_estimated_pose[1,0])/(QR_global_pose[1,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    Jacobean_Mat[9,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[2,1] - robot_estimated_pose[1,0])/(QR_global_pose[2,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[2,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[2,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[9,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[2,1] - robot_estimated_pose[1,0])/(QR_global_pose[2,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[2,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[2,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[9,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[2,1] - robot_estimated_pose[1,0])/(QR_global_pose[2,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    Jacobean_Mat[10,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[3,1] - robot_estimated_pose[1,0])/(QR_global_pose[3,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[3,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[3,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[10,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[3,1] - robot_estimated_pose[1,0])/(QR_global_pose[3,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[3,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[3,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[10,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[3,1] - robot_estimated_pose[1,0])/(QR_global_pose[3,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    Jacobean_Mat[11,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[4,1] - robot_estimated_pose[1,0])/(QR_global_pose[4,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[4,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[4,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[11,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[4,1] - robot_estimated_pose[1,0])/(QR_global_pose[4,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[4,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[4,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[11,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[4,1] - robot_estimated_pose[1,0])/(QR_global_pose[4,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    Jacobean_Mat[12,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[5,1] - robot_estimated_pose[1,0])/(QR_global_pose[5,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[5,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[5,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[12,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[5,1] - robot_estimated_pose[1,0])/(QR_global_pose[5,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[5,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[5,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[12,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[5,1] - robot_estimated_pose[1,0])/(QR_global_pose[5,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    Jacobean_Mat[13,0] =  f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[6,1] - robot_estimated_pose[1,0])/(QR_global_pose[6,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,1] - robot_estimated_pose[1,0]) / (pow(pow(QR_global_pose[6,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[6,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[13,1] =  -f_length * float(pow(mpmath.sec(math.atan((QR_global_pose[6,1] - robot_estimated_pose[1,0])/(QR_global_pose[6,0] - robot_estimated_pose[0,0])) - psyi),2) * (QR_global_pose[0,0] - robot_estimated_pose[0,0]) / (pow(pow(QR_global_pose[6,0] - robot_estimated_pose[0,0],2) + pow(QR_global_pose[6,1] - robot_estimated_pose[1,0],2),2)))
    Jacobean_Mat[13,2] = -f_length  * float(pow(mpmath.sec(math.atan((QR_global_pose[6,1] - robot_estimated_pose[1,0])/(QR_global_pose[6,0] - robot_estimated_pose[0,0])) - psyi),2)) 

    
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
var_center_pos = []

for row in sorted_data:
    curr_QR = row[1]
    
    if prev_QR == None:
        prev_QR = curr_QR
        #Create height and center pos buffers
        measured_height = []
        center_pos = []
    if prev_QR == curr_QR:
        # Fill the respective buffer
        measured_height.append(row[5])
        center_pos.append(row[2])
    else:
        #Measure the variance in measurement from the buffers
        var_measured_height.append(np.var(measured_height))
        var_center_pos.append(np.var(center_pos))
        avg_measured_height.append(np.mean(measured_height))
        avg_center_pos.append(np.mean(center_pos))        
        #clear the buffers
        measured_height.clear()
        center_pos.clear()
        prev_QR = curr_QR
        # add the height and center pos into the buffer
        measured_height.append(row[5])
        center_pos.append(row[2])
    
#Measure the last variance of measurement from buffer
var_measured_height.append(np.var(measured_height))
var_center_pos.append(np.var(center_pos))
avg_measured_height.append(np.mean(measured_height))
avg_center_pos.append(np.mean(center_pos))
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
R_Matrix = np.zeros((2,1))
#QR_Codes_with_var[:,0] = QR_codes
#QR_Codes_with_var[:,1] = var_center_pos
#QR_Codes_with_var[:,2] = var_measured_height
#QR_code_index = np.where(global_pos_Data == curr_QR)
#QR_global_pose = [global_pos_Data[QR_code_index[0][0],1],global_pos_Data[QR_code_index[0][0],2]]
initilization()
pose_x = []
pose_y = []
num_iterations = 3000

Total_data = np.zeros((7,7))
Total_data[:,0] = QR_codes
Total_data[:,1] = avg_measured_height
Total_data[:,2] = avg_center_pos
Total_data[:,3] = global_Xpos_QR
Total_data[:,4] = global_Ypos_QR
Total_data[:,5] = var_measured_height
Total_data[:,6] = var_center_pos       


QR_global_pose = np.array([global_Xpos_QR, global_Ypos_QR]).transpose() 
m= avg_measured_height+avg_center_pos
Measurement[:,0] = m


for i in range(num_iterations):
    update_direction = 0
       
        # QR_code_index = np.where(global_pos_Data == curr_QR)
        # QR_code_in_var = np.where(QR_Codes_with_var == curr_QR)
        # variance_in_measurement = [QR_Codes_with_var[QR_code_in_var[0][0],1],QR_Codes_with_var[QR_code_in_var[0][0],2]]
        # R_Matrix[0,0] = variance_in_measurement[0]
        # R_Matrix[1,0] = variance_in_measurement[1]
        # QR_global_pose = [global_pos_Data[QR_code_index[0][0],1],global_pos_Data[QR_code_index[0][0],2]]
    update_direction += Gradient_decent_cost_function(QR_global_pose,robot_estimated_pose,Measurement,R_Matrix)
    robot_estimated_pose = np.subtract(robot_estimated_pose , step_size * update_direction)
    
    pose_x.append(robot_estimated_pose[0])
    pose_y.append(robot_estimated_pose[1])

plt.plot(pose_x,label="P_x")
plt.plot(pose_y,label="P_y")
plt.ylabel("Position values")
plt.xlabel("iterations")
plt.legend()
plt.show()
print(time.time() - t0)
