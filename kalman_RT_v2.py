import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import smbus            #import SMBus module of I2C
from time import sleep          #import

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


# kalman filter model parameters
T = 10**(-3)
qw = 5
qb = 0.01
r_theta = 5*(10**(-5))
rw = 10**(-6)
Ad = np.array([[1, 0, 0], [T, 1, 0], [0, 0, 1]])
q_01 = (T ** 2) * qw / 2
Qd = np.array([[T * qw, q_01, 0], [q_01, (T ** 3) * qw / 3, 0], [0, 0, T * qb]])
C = np.array([[0, 1, 0], [1, 0, 1]])
I = np.identity(3)
R = np.array([[r_theta, 0], [0, rw]])
rad_sec = np.pi / 180


def initialise_kf():
    global T
    pw = 10
    pb = 10
    mu0 = np.zeros(shape=(3, 1))                   # np.array([0, 0, 0]) mu_0.shape = (3, 1)
    p_01 = (T ** 2) * pw / 2
    P0 = np.array([[T * pw, p_01, 0], [p_01, (T ** 3) * pw / 3, 0], [0, 0, T * pb]])
    return mu0, P0


def time_update_kf(mu_k_1, p_k_1):
    global Ad, Qd
    mu_k_k1 = np.dot(Ad, mu_k_1)
    d1 = np.dot(Ad, p_k_1)
    p_k_k1 = np.dot(d1, Ad.T) + Qd
    return mu_k_k1, p_k_k1


def measurement_correction(mu_k_k1, P_k_k1, yk):
    global R, C, I

    # update kalman gain
    S_1 = np.linalg.inv(np.linalg.multi_dot([C, P_k_k1, C.T]) + R)        # S_1 = (C * P_k_k1 * C.T + R)^(-1)
    K_gain = np.linalg.multi_dot([P_k_k1, C.T, S_1])                      # P_k_k1 * C.T * (S_1)

    # update mean
    err = yk - np.dot(C, mu_k_k1)
    mu_k_k1 = mu_k_k1 + np.dot(K_gain, err)

    # update covariance
    H_mat = I - np.dot(K_gain, C)
    P_k_k1 = np.linalg.multi_dot([H_mat, P_k_k1, H_mat.T]) + np.linalg.multi_dot([K_gain, R, K_gain.T])

    return mu_k_k1, P_k_k1


def MPU_Init():
    #write to Register 25 Sample Rate Divider
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)    # Sample_rate=gyro_out_rate/(1+SMPLRT_DIV) & gyro_out_rate = 8khz if CONFIG = xxxxx-000 or 111
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

mu, P = initialise_kf()
i = 0
while True:
    i+=1
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    gyro_x = read_raw_data(GYRO_XOUT_H)


    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    Gx = gyro_x/131.0
    
    y_theta = np.arctan2(Az, Ay)                  # y_theta output in radians
    yw = Gx * rad_sec                             # output in rad/sec


    mu, P = time_update_kf(mu, P)
    y = np.array([y_theta, yw])
    y.shape = (2, 1)
    mu, P = measurement_correction(mu, P, y)
    theta_dot = (mu[0] + mu[2]) / rad_sec   
    theta = (mu[1] / rad_sec) + 90
    if i % 100 == 0:
        print("Theta(deg): {0}\tGyro(deg/sec): {1}".format(theta, theta_dot))