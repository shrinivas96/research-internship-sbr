import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

T = 0.01
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


# mu = [mu_w; mu_theta; mu_b]'


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


rad_sec = np.pi / 180

df = pd.read_csv('example1.csv')
df2 = df.drop(['Time (sec)', 'Voltage (V)', 'Gyro45(deg/sec)', 'Theta (deg)'], axis=1)
accX = np.array(df['AccX (m/s^2)'].tolist())
accY = np.array(df['AccY (m/s^2)'].tolist())
gyro = np.array(df['Gyro (deg/sec)'].tolist())


y_theta = np.arctan2(-accY, accX)               # y_theta output in radians
yw = gyro * rad_sec                             # output in rad/sec

w45 = []                                        # empty list to record values of mu_w + mu_b
mu_theta = []                                   # empty list to record values of mu_theta


mu, P = initialise_kf()

for i in range(1000):
    mu, P = time_update_kf(mu, P)
    y_temp = np.array([y_theta[i], yw[i]])
    y_temp.shape = (2, 1)
    mu, P = measurement_correction(mu, P, y_temp)
    temp1 = (mu[0] + mu[2]) / rad_sec           # temp1 output in degrees
    w45.append(temp1)
    mu_theta.append(mu[1])

w45_arr = np.array(w45)

mu_theta_arr = (np.array(mu_theta) / rad_sec) + 90


# print(w45_arr)

df3 = df[:1000].copy()
plt.figure()
df3['w45_estimate'] = w45_arr
# df3['theta_estimate'] = mu_theta_arr
# df3['arctan2(y/x)'] = y_theta[:1000] / rad_sec
# df3['Theta2 (deg)'] = df3['Theta (deg)'] - 90

plt.plot(df3['Gyro45(deg/sec)'], label="Reference w45")
# plt.plot(df3['Theta (deg)'], label="Reference theta")
# plt.plot(df3['Theta2 (deg)'], label="Reference theta2")
# plt.plot(df3['arctan2(y/x)'], label="Arctan2")


plt.plot(df3['w45_estimate'], label="Predicted w45")
# plt.plot(df3['theta_estimate'], label="Predicted theta")

plt.legend(loc='best')
plt.show()
