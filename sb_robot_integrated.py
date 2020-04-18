import kalman_filter as kf
import AMIS30543 as amis
import RPi.GPIO as GPIO
from time import sleep
from mpu6050 import *
import numpy as np
import pigpio
import spidev
import smbus
import sys
import csv
import os

# addresses for reading vaues of acc x, y, z and gyro x, y, z respectively
# this file uses only acc x, z and gyro y due to placement of IMU
# if your IMU is placed differently change the addresses in read_raw_data
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47


def Shutdown(channel):
    # currently we turn the Pi off. to improve, change this function with a way to turn off the program
    # and return back to 0 initialised states waiting to be started again
    print("Shutting Down")
    # the motor keeps on spinning even after ports closed, unless the freq & duty cycle not 0
    pwm_obj.hardware_PWM(STEP, 0, 0)
    pwm_obj.hardware_PWM(STEP2, 0, 0)
    debug_file.close()
    stepper1.close_port()
    GPIO.cleanup()
    sleep(5)
    os.system("sudo shutdown -h now")
    input()                                                     # for stalling the function till shutdown is executed


def linear_matrices():
    r = 0.045
    m = 1
    M = 1.2
    b = 0.01
    c = 0.0047
    J = 5e-6
    I = 1e-3
    alpha = 0
    g = -9.8
    l = 0.188

    n1_32 = M*g*l*(J + 2*M*(r**2) + 2*m*(r**2) - M*(r**2)*(np.cos(alpha)**2) - m*(r**2)*(np.cos(alpha)**2) + M*l*r*np.cos(alpha))
    d1_32 = M**2*l**2*r**2 - M**2*l**2*r**2*(np.cos(alpha)**2) + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
    n2_32 = 4*M**2*l**2*r**3*g*np.cos(alpha)*np.sin(alpha)*(M+m)*(2*M*np.sin(alpha)*l**2 + 2*M*r*l*np.cos(alpha)*np.sin(alpha) +2*M*r*l*np.cos(alpha)*np.sin(alpha) + 2*I*np.sin(alpha));
    d2_32 = (2*I*J + M**2*l**2*r**2 + 2*J*M*l**2 + 2*I*M*r**2 + 2*I*m*r**2 - (M**2*l**2*r**2)*(2*np.cos(alpha)**2 - 1) + 2*M*(l**2)*m*(r**2))**2

    a32 = (n1_32/d1_32) + (n2_32/d2_32)

    n33 = b*(I + J + M*l**2 + M*r**2 + m*r**2 + 2*M*l*r*np.cos(alpha))
    d33 = M**2*l**2*r**3 - M**2*l**2*r**3*np.cos(alpha)**2 + m*M*l**2*r**3 + J*M*r*l**2 + I*M*r**3 + I*m*r**3 + I*J*r
    a33 = -n33/d33

    n34 = I*b + J*b - c*m*r**3 - J*c*r + M*b*l**2 + M*b*r**2 - M*c*r**3 + b*m*r**2 - M*c*l*r**2*np.cos(alpha) + 2*M*b*l*r*np.cos(alpha)
    d34 = r*(M**2*l**2*r**2 - M**2*l**2*r**2*(np.cos(alpha)**2) + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J)
    a34 = -n34/d34

    n1_42 = M*g*l*(J + M*r**2 + m*r**2 + M*r**2*np.sin(alpha)**2 + m*r**2*np.sin(alpha)**2)
    d1_42 = M**2*l**2*r**2*np.sin(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
    n2_42 = 2*M**3*g*l**3*r**4*np.sin(2*alpha)**2*(M + m)
    d2_42 = (2*I*J + M**2*l**2*r**2 + 2*J*M*l**2 + 2*I*M*r**2 + 2*I*m*r**2 + 2*M*l**2*m*r**2 - M**2*l**2*r**2*np.cos(2*alpha))**2
    a42 = (-n1_42/d1_42) - (n2_42/d2_42)

    n43 = J*b + M*b*r**2 + b*m*r**2 + M*b*l*r*np.cos(alpha)
    d43 = r*(M**2*l**2*r**2 - M**2*l**2*r**2*np.cos(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J)
    a43 = n43/d43

    n44 = J*b - c*m*r**3 - J*c*r + M*b*r**2 - M*c*r**3 + b*m*r**2 + M*b*l*r*np.cos(alpha)
    d44 = r*(M**2*l**2*r**2 - M**2*l**2*r**2*(np.cos(alpha)**2) + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J)
    a44 = n44/d44

    linA = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, a32, a33, a34], [0, a42, a43, a44]])

    nb3 = I + J + M*l**2 + M*r**2 + m*r**2 + 2*M*l*r*np.cos(alpha)
    db3 = M**2*l**2*r**2 - M**2*l**2*r**2*np.cos(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
    b3 = nb3/db3

    nb4 = J + M*r**2 + m*r**2 + M*l*r*np.cos(alpha)
    db4 = M**2*l**2*r**2 - M**2*l**2*r**2*np.cos(alpha)**2 + m*M*l**2*r**2 + J*M*l**2 + I*M*r**2 + I*m*r**2 + I*J
    b4 = nb4/db4

    linB = np.array([0, 0, b3, b4])             # the current shape is (4,)
    return linA, linB


def controller(ctrl_input):
    # different controllers were used with varying outputs. more details in readme
    # 1 K = np.array([1.5275, 7.1543, 0.0884, 0.4621])  # for Q = diag([7e0, 9e1, 5e-2, 3e-1]), R = 3e0
    # 2 K = np.array([0.0408, 5.2649, 0.0623, 0.4309])  # for Q = diag([5e-3, 9e1, 5e-6, 3e-1]), R = 3e0
    # 3 K = np.array([0.0187, 0.1336, 0.0118, 0.0580])  # for Q = diag([7e-2, 9e1, 9e-3, 3e-1]), R = 2e2
    # 4 K = np.array([0.3162, 1.1003, 0.0617, 0.3112])  # for Q = diag([10, 50, 1, 5]), R = 100
    K = np.array([0.0024, 0.8524, 0.0213, 0.116])       # for Q = diag([5e-4, 9e1, 2e-3, 3e-1]), R = 90

    y = -(K[0] * ctrl_input[0] + K[1] * ctrl_input[1] + K[2] * ctrl_input[2] + K[3] * ctrl_input[3])
    # needs to be updated to add A, B, C matrices. Currently, y = -Kz
    return y


def step_forward(frequency):
    GPIO.output(DIR, GPIO.HIGH)
    GPIO.output(DIR2, GPIO.HIGH)
    var_m1 = pwm_obj.hardware_PWM(STEP, frequency, 500000)
    var_m2 = pwm_obj.hardware_PWM(STEP2, frequency, 500000)
    # if the motors do not work look at these variables. variable value 0 if executed normally
    return var_m1, var_m2


def step_backward(frequency):
    GPIO.output(DIR, GPIO.LOW)
    GPIO.output(DIR2, GPIO.LOW)
    var_m1 = pwm_obj.hardware_PWM(STEP, frequency, 500000)
    var_m2 = pwm_obj.hardware_PWM(STEP2, frequency, 500000)
    # if the motors do not work look at these variables. variable value 0 if executed normally
    return var_m1, var_m2


pwm_obj = pigpio.pi()

button = 37                                     # button to shutdown the pi.


SlaveSelect1 = 33                               # different SS pin because we're not able to control the actual SS pin with spidev library
SlaveSelect2 = 29                               # connect the ss pin of the slave to below pin and not to SPI0 CE1
DIR = 13
DIR2 = 31
STEP = 18                                       # BCM GPIO 18. As PWM is supported on that pin. Setting mode as BCM not needed
STEP2 = 12                                      # BCM GPIO 12. As PWM is supported on that pin. Setting mode as BCM not needed

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SlaveSelect1, GPIO.OUT)
GPIO.setup(SlaveSelect2, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setwarnings(False)
GPIO.add_event_detect(button, GPIO.FALLING, callback=Shutdown, bouncetime=2000)

# initializing the stepper motor driver and motor parameters
maxCurrent = 1800                                               # unit miliamps. change according to your motor specifications
steps_per_rev = 200                                             # total steps for 1 rev of motor. change according to your motor specifications
u_steps = 16                                                    # microstepping mode u_step mi 16

m1_spi_bus = 0
m1_spi_dev = 1

m2_spi_bus = 1
m2_spi_dev = 1

stepper1 = amis.AMIS30543(m1_spi_bus, m1_spi_dev)
stepper1.initSS(SlaveSelect1)
pwm_obj.write(STEP, 0)                                          # same as GPIO.output(STEP, GPIO.LOW)

stepper2 = amis.AMIS30543(m2_spi_bus, m2_spi_dev)
stepper2.initSS(SlaveSelect2)
pwm_obj.write(STEP2, 0)

GPIO.output(DIR, GPIO.LOW)
GPIO.output(DIR2, GPIO.LOW)
sleep(1)                                                        # giving the driver some time to power up.

stepper1.reset_settings()
stepper1.setCurrentMilliamps(maxCurrent)
stepper1.setStepMode(u_steps)
stepper1.enableDriver()

stepper2.reset_settings()
stepper2.setCurrentMilliamps(maxCurrent)
stepper2.setStepMode(u_steps)
stepper2.enableDriver()

theta_arr = []
torque_arr = []
phi_dot_arr = []
freq_arr = []
phi_arr = []

t_step = 1e-3                                                           # the time step for control loop.
deg2rad = np.pi / 180
rad2deg = 180 / np.pi
linA, linB = linear_matrices()
tA = t_step * linA
tB = t_step * linB

# recording all the states for future debugging
debug_file = open("debug.csv", 'w', newline='')
obj = csv.writer(debug_file)

# initializing states for first run
state_arr = np.array([0, 0, 0, 0], dtype='float64').reshape((4, 1))     # states = [phi, theta, phi_dot, theta_dot]

# initializing the imu and the kalman filter
# kalman filter only estimates theta and theta_dot
MPU_Init()
mu, P = kf.initialise_kf()                                              # mu = [mu_dtheta, mu_theta, mu_bias]
i = 0

while True:
    tau = controller(state_arr)                                         # tau unit Nm (Newton.meter) controller implements u = -Kx

    # for an explanation of phi_dot and phi formulation please refer to readme
    # phi_dot unit rad/sec. convert to rev/sec for control input
    phi_dot = state_arr[2] + np.matmul(tA, state_arr)[2] + tB[2] * tau  # dphi(t+h) = dphi(t) + (Ax)[2] + (Bu)[2]
    phi = (phi_dot * t_step) + state_arr[0]  # unit rad                 # phi(t+h) = phi(t) + h*dphi(t+h)

    two_dphi_rev = phi_dot / np.pi                                      # unit rev/sec. for applying phi_dot to motors

    freq = two_dphi_rev * steps_per_rev * u_steps                       # freq is directly the value for pigpio PWM freq
    int_freq = abs(int(freq))                                           # as hardware_PWM() cannot execute floating point PWM values

    if tau < 0:
        step_backward(int_freq)

    elif tau >= 0:
        step_forward(int_freq)

    # reading the raw IMU values
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    Ax = acc_x / 16384.0
    Az = acc_z / 16384.0
    Gy = gyro_y / 131.0

    # Ax, Az, Gy = read_new_value()
    y_theta = np.arctan2(Az, Ay)                                        # y_theta = -(-Az/Ay). output in radians
    yw = Gx * deg2rad                                                   # convert to rad/sec as input in deg/sec

    mu, P = kf.time_update_kf(mu, P)                                    # predicted mean, covariance based on process model
    y = np.array([y_theta, yw]).reshape((2, 1))                         # measurement array
    mu, P = kf.measurement_correction(mu, P, y)                         # mu, P correction
    theta_dot = mu[0]                                                   # output in rad/sec. originally mu[0] + mu[2]
    theta = (mu[1]) + (np.pi / 2)  # + 0.026                            # theta output in radians

    # array containing all variables of interest to write to file
    # append tau and freq to a copy of state array. preserves the original states array
    wrt_arr = np.append(state_arr.copy(), [tau, int_freq])
    obj.writerow(wrt_arr)

    # updating all the states
    state_arr[0] = phi
    state_arr[1] = theta
    state_arr[2] = phi_dot
    state_arr[3] = theta_dot
