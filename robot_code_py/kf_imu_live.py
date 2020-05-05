import numpy as np
from mpu6050 import *
import kalman_filter as kf

"""
bug: trying to print the estimated values in the while loop causes the entire
program and raspberry pi to hang. adding a count variable 'c' and then printing 
only after every 100 iteration seems to work for now. no idea why!

possible reason: reading a new value and estimating values takes much less time 
than executing a print statement.
"""

deg2rad = np.pi / 180
rad2deg = 1 / deg2rad

MPU_Init()

c = 0

# mu = [mu_w; mu_theta; mu_b]^T
# estimating w -> dtheta, theta, b -> bias
# initialising for t = 0
mu, P = kf.initialise_kf()

# recursively iterate over all data points
while True:
    c += 1
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    gyro_x = read_raw_data(GYRO_XOUT_H)

    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    Gx = gyro_x / 131.0

    y_theta = np.arctan2(Az, Ay)                                            # y_theta = -(-Az/Ay), because of imu placement. output in radians
    yw = Gx * deg2rad                                                       # convert to rad/sec as input in deg/sec

    mu, P = kf.time_update_kf(mu, P)                                        # prediction based on process model
    y_arr = np.array([y_theta, yw]).reshape((2, 1))                         # y_arr = y = [y_theta, y_w]^T
    mu, P = kf.measurement_correction(mu, P, y_arr)                         # mu, P estimate based on measurement y_arr

    dtheta_temp = (mu[0] + mu[2]) * rad2deg                                 # output in deg/sec
    # dtheta_temp = (mu[0] + mu[2])                                         # output in rad/sec
    theta_temp = (mu[1] + np.pi / 2) * rad2deg                              # output in deg
    # theta_temp = mu[1] + np.pi/2                                          # output in deg

    dtheta_deg = (mu[0] + mu[2]) * rad2deg                                  # output in deg/sec
    # dtheta_rad = (mu[0] + mu[2])                                          # output in rad/sec
    theta_deg = (mu[1] * rad2deg) + 90                                      # note to self: check with live imu data why was +90 added?
    if c % 100 == 0:
        print("Theta(deg): {0}\tGyro(deg/sec): {1}".format(theta_deg, dtheta_deg))
