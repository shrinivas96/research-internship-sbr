import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import kalman_filter as kf

deg2rad = np.pi / 180
rad2deg = 1 / deg2rad

input_df = pd.read_csv('example1.csv')
df_len = len(input_df)
# input_data_df = input_df.drop(['Time (sec)', 'Voltage (V)', 'Gyro45(deg/sec)', 'Theta (deg)'], axis=1)

accX = np.array(input_df['AccX (m/s^2)'].tolist())                          # converting to array form to 
accY = np.array(input_df['AccY (m/s^2)'].tolist())                          # iterate over each element
gyro = np.array(input_df['Gyro (deg/sec)'].tolist())                        # to bring in form of measurement 'y'


y_theta = np.arctan2(-accY, accX)                                           # y_theta = -ay/ax. output in radians
yw = gyro * deg2rad                                                         # convert to rad/sec as input in deg/sec

dtheta_pred = np.zeros(df_len)                                              # empty list to record values of mu_w + mu_b
theta_pred = np.zeros(df_len)                                               # empty list to record values of mu_theta

# mu = [mu_w; mu_theta; mu_b]^T
# estimating w -> dtheta, theta, b -> bias
# initialising for t = 0
mu, P = kf.initialise_kf()

# iterate recursively over all data points
for i in range(df_len):
    mu, P = kf.time_update_kf(mu, P)                                        # prediction based on process model
    y_arr = np.array([y_theta[i], yw[i]]).reshape((2, 1))                   # y_arr = y = [y_theta, y_w]^T
    mu, P = kf.measurement_correction(mu, P, y_arr)                         # mu, P estimate based on measurement y_arr

    # temporary variables to hold the values of dtheta and theta
    # improvement: name variables according to the values they hold
    dtheta_temp = (mu[0] + mu[2]) * rad2deg                                 # output in deg/sec
    # dtheta_temp = (mu[0] + mu[2])                                         # output in rad/sec
    theta_temp = (mu[1] + np.pi/2) * rad2deg                                # output in deg
    # theta_temp = mu[1] + np.pi/2                                          # output in deg

    dtheta_pred[i] = dtheta_temp                                            # check unit of both arrays based on \
    theta_pred[i] = theta_temp                                              # which unit is used from above estimate


# validation df only contains columns corresponding to final values. to validate estimated vs reference
# columns: Gyro45(deg/sec), Theta (deg), dtheta_estimate, theta_estimate
validation_df = input_df.copy().drop(['Voltage (V)', 'AccX (m/s^2)', 'AccY (m/s^2)', 'Gyro (deg/sec)'], axis=1)
validation_df['dtheta_estimate'] = dtheta_pred
validation_df['theta_estimate'] = theta_pred

# shows all values without applying the filter. check if this is the same theta as after applying the filter. it's not
# validation_df['arctan2(y/x)'] = (y_theta + np.pi/2) * rad2deg 

plt.figure(figsize=(22, 10))
# plt.plot(validation_df['Time (sec)'], validation_df['Theta (deg)'], label="Reference theta")
plt.plot(validation_df['Time (sec)'], validation_df['Gyro45(deg/sec)'], label="Reference dtheta")

# plt.plot(validation_df['Time (sec)'], validation_df['arctan2(y/x)'], label="arctan2 theta")
# plt.plot(validation_df['Time (sec)'], validation_df['theta_estimate'], label="Predicted theta")
plt.plot(validation_df['Time (sec)'], validation_df['dtheta_estimate'], label="Predicted dtheta")

plt.legend(loc='best')
plt.show()
