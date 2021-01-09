# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 14:25:57 2021

@author: AB
"""
import numpy as np
import math , mpmath
import matplotlib.pyplot as plt
import time

   
X = np.zeros((4,1))


def A_mat(phi,dt):
    
    A = np.zeros((4,4))
    A[0,0] = 1
    A[1,1] = 1
    A[2,2] = 1
    A[3,3] = 1
    A[0,2] = dt * np.cos(phi)
    A[1,2] = dt * np.sin(phi)

    return A

def B_mat(dt):
    B = np.zeros((4,1))
    B[3,0] = dt
    return B

def state(x,y,phi):
    global X 
    X[0,0] = x
    X[1,0] = y
    X[2,0] = 0
    X[3,0] = phi 

def stateupdate(dt,phi):
    
    A_m = A_mat(X[3,0],dt)
    B_m = B_mat(dt)
    
    updated_state = np.matmul(A_m,X) + B_m*phi 
    updated_state[3,0] -= np.deg2rad(0.1873) 
    
    return updated_state
   
#file_name = "Task6/camera_tracking_task6.csv"
#data = csv.reader(file_path+file_name,delimiter=',')
#camera_tracking = np.genfromtxt(file_name,delimiter=',')

file_name = "Task6/imu_tracking_task6.csv"
#data = csv.reader(file_path+file_name,delimiter=',')
imu_tracking = np.genfromtxt(file_name,delimiter=',')

file_name = "Task6/motor_tracking_task6.csv"
#data = csv.reader(file_path+file_name,delimiter=',')
motor_tracking = np.genfromtxt(file_name,delimiter=',')


p_x = 16.5
p_y = 49.8
phi = 1.5707
P_x_arr = []
P_y_arr = []
phi_arr = []


state(p_x,p_y,phi)



avg_gyro = []
i =0
for ind in range(253):
    avg_gyro.append(np.deg2rad((np.sum(imu_tracking[i:i+8,8])/8)))
    i +=8 

avg_gyro = np.hstack([np.zeros(14),avg_gyro])
V = []
for row in motor_tracking:
     V.append(10 * (row[1] + row[2])) 
    
for i in range(len(avg_gyro)):
     X[2,0] = V[i]
     X =  stateupdate(0.5,avg_gyro[i])
#     #if not len(prev_row):
#     #     prev_row = curr_row
#     # continue

#     curr_time = curr_row[0]
#     # prev_time = prev_row[0]    
#     [index,val] = min(enumerate(motor_tracking[:,0]), key=lambda x: x[1] if x[1] > curr_time else float('inf'))
#     V = 10 * (motor_tracking[index,1] + motor_tracking[index,2])
#     X[2,0] = V
#     #if imu_tracking[imu_time_index,0] < curr_time:
        
#     gyro = curr_row[8]
#     X =  stateupdate(0.063,gyro)
            
#     prev_time = curr_time
     P_x_arr.append(X[0,0])
     P_y_arr.append(X[1,0])
     phi_arr.append(X[3,0])
    
plt.scatter(P_x_arr,P_y_arr)

plt.plot(P_y_arr)
plt.plot(V*np.cos(phi_arr))

    
           
        
        
    