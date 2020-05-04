import numpy as np

"""
Throughout the code
(.)_k_1 refers to value at time k-1
(.)_k_k1 refers to value at time k give k-1
could possibly come up with better naming
"""

# Initialising constants for the Kalman Filter
T = 0.01

# R -> measurement covariance
rw = 10 ** (-6)
r_theta = 5 * (10 ** (-5))
R = np.array([[r_theta, 0], [0, rw]])

# Q -> motion covariance
qw = 5
qb = 0.01
q_01 = (T ** 2) * qw / 2
Qd = np.array([[T * qw, q_01, 0], [q_01, (T ** 3) * qw / 3, 0], [0, 0, T * qb]])

# Ad -> state transition model. C -> measurement matrix
Ad = np.array([[1, 0, 0], [T, 1, 0], [0, 0, 1]])
C = np.array([[0, 1, 0], [1, 0, 1]])
I = np.identity(3)


# Initialising the mean and covariance estimate. 
def initialise_kf():
    # initialise mean theta esitimate to be -pi/2 because of placement of IMU.
    # not defining mu as a column vector causes matrix multiplication error in measurement correction
    mu0 = np.array([0, -np.pi / 2, 0]).reshape((3, 1))

    pw, pb = 10, 10
    p_01 = (T ** 2) * pw / 2
    P0 = np.array([[T * pw, p_01, 0], [p_01, (T ** 3) * pw / 3, 0], [0, 0, T * pb]])
    return mu0, P0


def time_update_kf(mu_k_1, p_k_1):
    # Prediction step of KF -> estimated mean and covariance

    mu_k_k1 = np.dot(Ad, mu_k_1)
    p_k_k1 = Ad.dot(p_k_1).dot(Ad.T) + Qd                               # P = Ad * P_(k-1) * Ad^T + Qd

    # output is essentially mu_bar, P_bar
    return mu_k_k1, p_k_k1


def measurement_correction(mu_k_k1, P_k_k1, yk):
    # update kalman gain
    S_inv = np.linalg.inv(np.linalg.multi_dot([C, P_k_k1, C.T]) + R)    # S_inv = (C * P_k_k1 * C.T + R)^(-1)
    K_gain = np.linalg.multi_dot([P_k_k1, C.T, S_inv])                  # K_gain = P_k_k1 * C.T * (S_inv)

    # update mean
    err = yk - np.dot(C, mu_k_k1)                                       # diff bw measurement and expected measrement
    mu_k_k1 = mu_k_k1 + np.dot(K_gain, err)

    # update covariance
    H_mat = I - np.dot(K_gain, C)                                       # H_mat = I - KC
    P_k_k1 = np.linalg.multi_dot([H_mat, P_k_k1, H_mat.T]) \
             + np.linalg.multi_dot([K_gain, R, K_gain.T])

    return mu_k_k1, P_k_k1


if __name__ == "__main__":
    print("The file contains only the main KF algorithm, prediction correction step\n")
    print("This model works only for estimating theta and theta_dot from the data. more info in the readme file")
