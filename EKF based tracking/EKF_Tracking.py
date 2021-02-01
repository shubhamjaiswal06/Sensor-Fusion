# -*- coding: utf-8 -*-
"""
Created on Fri Jan  8 16:52:30 2021

@author: AB
"""
"""

Extended kalman filter (EKF) localization sample

First author: Atsushi Sakai (@Atsushi_twi)

"""

import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Covariance for EKF simulation
Q = np.diag([
    1,  # variance of location on x-axis
    1,  # variance of location on y-axis
    5*np.deg2rad(1.0),  # variance of yaw angle
])  # predict state covariance
R = np.array([1,0.5])  # Observation x,y position covariance
actual_height = 11.5
f_length = 524


DT = 0 # time tick [s]


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * np.cos(x[2, 0]), 0],
                  [DT * np.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = F @ x + B @ u

    return x

def measurement_model(x,QR_pos):
    g_x = np.zeros((2*QR_pos[:,0].size,1))
    
    for i in range(QR_pos[:,0].size):
        g_x[i*2] = actual_height * f_length /(np.sqrt(pow(QR_pos[i,0] - x[0,0],2) + pow(QR_pos[i,0] - x[1,0],2)))
        g_x[i*2+1] = f_length * np.tan(np.arctan((QR_pos[i,0] - x[1,0])/(QR_pos[i,0] - x[0,0])) - x[2,0])

    return g_x


def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dy/dyaw = v*dt*cos(yaw)
    """
    jF = np.array(np.diag([1,1,1]),dtype = float)
    yaw = x[2, 0]
    v = u[0, 0]
    jF[0,2] = -DT * v * math.sin(yaw)
    jF[1,2] = DT * v * math.cos(yaw)
    # jF = np.array([
    #     [1.0, 0.0, -DT * v * math.sin(yaw)],
    #     [0.0, 1.0, DT * v * math.cos(yaw)],
    #     [0.0, 0.0, 0.0, 1.0]])

    return jF

def H_matrix(x):
    
    H = np.diag(np.ones(2*QR_pos[:,0].size))
    
    return H@x

def jacob_h(x,QR_pos):
    
    ##This function creates the jacobean matrix for calculations ##
    # For the current measurement model, 2 rows and 3 colomns jacobean will be created
    #Jacobean_Mat = np.zeros((7,2))  # Initial Jacobean Matrix
    Jh = np.zeros((2*QR_pos[:,0].size,3))
    
    
    
    for i in range(QR_pos[:,0].size):
        
        Jh[i*2,0] = actual_height * f_length * (QR_pos[i,0] - x[0,0]) / (pow(pow(QR_pos[i,0] - x[0,0],2) + pow(QR_pos[i,1] - x[1,0],2),3/2))
        Jh[i*2,1] = actual_height * f_length * (QR_pos[i,1] - x[1,0]) / (pow(pow(QR_pos[i,0] - x[0,0],2) + pow(QR_pos[i,1] - x[1,0],2),3/2))
        #Jacobean_Mat[i*2,2] = np.array(-actual_height * f_length * (QR_pos[:,1] - x[1,0]) / (pow(pow(QR_pos[:,0] - x[0,0],2) + pow(QR_pos[:,1] - x[1,0],2),3/2)))
        
        D = (pow(pow(QR_pos[i,0] - x[0,0],2) + pow(QR_pos[i,1] - x[1,0],2),2))
        Jh[i*2+1,0] =   f_length * ((pow(np.tan(np.arctan((QR_pos[i,0] - x[1,0])/(QR_pos[i,0] - x[0,0]))),2) + 1) - x[2,0]) * (QR_pos[i,0] - x[0,0]) /D
        Jh[i*2+1,1] = -  f_length * ((pow(np.tan(np.arctan((QR_pos[i,0] - x[1,0])/(QR_pos[i,0] - x[0,0]))),2) + 1) - x[2,0]) * (QR_pos[i,1] - x[1,0]) /D
        Jh[i*2+1,2] = -  f_length * ((pow(np.tan(np.arctan((QR_pos[i,0] - x[1,0])/(QR_pos[i,0] - x[0,0]))),2) + 1) - x[2,0])
    return Jh


def ekf_prediction(xEst, PEst, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    return xPred, PPred

def ekf_estimation(xPred, PPred,z_m,QR_pos,QR_data):
    #  Update
    jH = jacob_h(xPred,QR_pos)
    zPred = measurement_model(xPred,QR_pos)
    y = z_m - zPred
    R_err = np.diag(np.kron(np.ones(QR_pos[:,0].size),R)) 
    S = jH @ PPred @ jH.T + R_err
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    
    return xEst,PEst

def imu_measurement(zm,xpre):
    
    zpred = xpre[3,0]
    while (zpred <= -np.pi/2): 
        zpred += np.pi
    while (zpred > np.pi/2):
        zpred -= np.pi
    y = zm - zpred
    r_err = np.deg2rad(2.0)
    
    
    
    return 
file_path = "E:/ashutosh/sensorfusionstuff/shubham project/Localization and Tracking/Sensor-Fusion/"

file_name = "Task6/camera_tracking_task6.csv"
camera_tracking = np.genfromtxt(file_path+file_name,delimiter=',')

file_name = "Task6/imu_tracking_task6.csv"
imu_tracking = np.genfromtxt(file_path+file_name,delimiter=',')

file_name = "Task6/motor_tracking_task6.csv"
motor_tracking = np.genfromtxt(file_path+file_name,delimiter=',')

#read the global QR position file
file_path = "E:/ashutosh/sensorfusionstuff/shubham project"
file_name = "qr_code_position_in_global_coordinate.csv"
global_pos_Data = np.genfromtxt(file_path+'/'+file_name,delimiter=',')
global_pos_Data = global_pos_Data[1:,:]

imu_index = 0
motor_index = 0
camera_index = 0

SIM_TIME = max(motor_tracking[motor_tracking[:,0].size-1,0],imu_tracking[imu_tracking[:,0].size-1,0],camera_tracking[camera_tracking[:,0].size-1,0])

PREV_TIME = min(motor_tracking[0,0],imu_tracking[0,0],camera_tracking[0,0])

CUR_TIME = 0

# State Vector [x y yaw]'

xEst = np.array([[16],[48],[1.50]])
xTrue = np.zeros((3, 1))
PEst = np.eye(3)

xDR = np.zeros((3, 1))  # Dead reckoning
u = np.zeros((2,1)) # velocity and yaw angle
# history
hxEst = xEst
hxTrue = xTrue
hxDR = xTrue
hz = np.zeros((2, 1))
z = np.zeros((2,1))

while SIM_TIME > CUR_TIME:
    
    
    if (imu_index <=imu_tracking[:,0].size-1 and imu_tracking[imu_index,0]<min(motor_tracking[motor_index,0],camera_tracking[camera_index,0])):
        CUR_TIME = imu_tracking[imu_index,0]
        
        DT = CUR_TIME - PREV_TIME
        xEst, PEst = ekf_prediction(xEst, PEst, u)
        
        u[1,0] = np.deg2rad(imu_tracking[imu_index,8]) - np.deg2rad(0.4)
        while (u[1,0] <= -np.pi/2): 
            u[1,0] += np.pi
        while (u[1,0] > np.pi/2):
            u[1,0] -= np.pi
        
        hxEst = np.hstack((hxEst, xEst))
        imu_measurement(u[1,0],xEst)
        imu_index +=1
        PREV_TIME = CUR_TIME
    
    elif(motor_index <=motor_tracking[:,0].size-1 and motor_tracking[motor_index,0]<min(imu_tracking[imu_index,0],camera_tracking[camera_index,0])):
            
            CUR_TIME = motor_tracking[motor_index,0]
        
            DT = CUR_TIME - PREV_TIME
            xEst, PEst = ekf_prediction(xEst, PEst, u)
            
            V = 10 * (motor_tracking[motor_index,1]  + motor_tracking[motor_index,2])
            u[0,0] = V 
        
            hxEst = np.hstack((hxEst, xEst))
            motor_index +=1
            PREV_TIME = CUR_TIME
    else:
        
        CUR_TIME = camera_tracking[camera_index,0]
        
        PREV_TIME = CUR_TIME
        
        DT = CUR_TIME - PREV_TIME
        xEst, PEst = ekf_prediction(xEst, PEst, u)
        
        counter = 0
        QR_data = []
        timer = 0
        
        while timer == 0:
            if CUR_TIME == camera_tracking[camera_index+counter,0]:
               counter +=1
            else:
               timer = 1
        QR_data = camera_tracking[camera_index:camera_index+counter,1:]
        
        Cx = QR_data[:,1]
        hi = QR_data[:,4]
        
        z_m = np.zeros((2*QR_data[:,0].size,1))
        for i in range(QR_data[:,0].size):
            z_m[i*2] = QR_data[i,4]
            z_m[i*2+1] = QR_data[i,1]
        
        
        #xPred, PPred = ekf_prediction(xEst, PEst, u)
        
        QR_pos = global_pos_Data[QR_data[:,0].astype('int'),1:3]
        

        xEst, PEst = ekf_estimation(xEst, PEst,z_m,QR_pos,QR_data)

        camera_index +=camera_index+counter
