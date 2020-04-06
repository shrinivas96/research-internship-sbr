import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import smbus
from time import sleep        
import spidev
import RPi.GPIO as GPIO


SlaveSelect = 33
DIR = 31
STEP = 29

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SlaveSelect, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setwarnings(False)





# motor driver class containing functions

class AMIS30543:
    def __init__(self):
        self.wr = 0
        self.cr0 = 0 
        self.cr1 = 0 
        self.cr2 = 0 
        self.cr3 = 0
        self.spi = spidev.SpiDev()    # Enable SPI
#         bus, device = 0, 1    #Device is the chip select pin. Set to 0 or 1, depending on the connections
        self.spi.open(0, 1)    # Open a connection to a specific bus and device (chip select pin)
        self.spi.max_speed_hz = 500000    # Set SPI speed and mode
        self.spi.mode = 0
        self.spi.lsbfirst = False

        
        self.REG = {'WR':  0x00,
                   'CR0': 0x01,
                   'CR1': 0x02,
                   'CR2': 0x03,
                   'CR3': 0x09,
                   'SR0': 0x04,
                   'SR1': 0x05,
                   'SR2': 0x06,
                   'SR3': 0x07,
                   'SR4': 0x0A}
        
        self.CMD = {'READ': 0x00, 
                    'WRITE':0x80}
    
    def reset_settings(self):
        self.wr = 0
        self.cr0 = 0 
        self.cr1 = 0 
        self.cr2 = 0 
        self.cr3 = 0
        self.applysettings()
    
    def initSS(self, ssPin):
        self.ssPin = ssPin
        GPIO.output(self.ssPin, GPIO.HIGH)
        
    def enableDriver(self):
        self.cr2 = self.cr2 | 0b10000000
        self.applysettings()
    
    def applysettings(self):
        self.writeReg(self.REG['CR2'], self.cr2)
        self.writeReg(self.REG['WR'], self.wr)
        self.writeReg(self.REG['CR0'], self.cr0)
        self.writeReg(self.REG['CR1'], self.cr1)
        self.writeReg(self.REG['CR3'], self.cr3)
    
    def writeReg(self, address, value):
        self.selectChip()
        byte1 = self.CMD['WRITE'] | (address & 0b11111)
        byte2 = value
#         Uncommment for poor man's debugging
#         print("Byte1[CMD,ADDR]:", byte1)
#         print("Byte2[DATA]:", byte2)
        self.transfer([byte1, byte2])
        self.deselctChip()
        
    def selectChip(self):
        GPIO.output(self.ssPin, GPIO.LOW)

    def deselctChip(self):
        GPIO.output(self.ssPin, GPIO.HIGH)
        # NEED TO ADD A MICROSECOND DELAY BECAUSE 
        # CS high time is specified as 2.5 us in the AMIS-30543 datasheet.
    
    def close_port(self):
        self.spi.close()
    
    def transfer(self, byteVal):
        receivedVal = self.spi.xfer2(byteVal)
#         Uncommment for poor man's debugging
#         print("Value received after transfer:\n", 
#               receivedVal, type(receivedVal), len(receivedVal))

    def setCurrentMilliamps(self, current):
        CUR_reg = 0
        # From Table 13 of the AMIS-30543 datasheet,
        # More values should be added for more current possibilities
        if current >= 2070:
            CUR_reg = 0b10100
        elif current >= 1850:
            CUR_reg = 0b10011
        elif current >= 1695:
            CUR_reg = 0b10010
        elif current >= 1520:
            CUR_reg = 0b10001   
#        print("Value of cr0 before setting CUR_reg:", bin(self.cr0))
        self.cr0 = (self.cr0 & 0b11100000) | CUR_reg;
#        print("Value of cr0 after setting CUR_reg:", bin(self.cr0))
        self.writeReg(self.REG['CR0'], self.cr0)
    
    def bit_not(self, n, numbits=8):
        return (1 << numbits) - 1 - n
    
    def setStepMode(self, mode):
        # By default mode is 32. 
        # See Table 12 fo AMIS-30543 datasheet
        esm = 0b000
        sm = 0b000
        if mode == 64:
            esm = 0b010
        elif mode == 128:
            esm = 0b001
        elif mode == 16:
            sm = 0b001
        else:
            print("Default mode set to 1/32 microsteps")
            
#        print("Value of cr0 before setting stepping mode:", bin(self.cr0))
        self.cr0 = (self.cr0 & self.bit_not(0b11100000, 8)) | (sm << 5)
#        print("Value of cr0 after setting CUR_reg:", bin(self.cr0))
        self.cr3 = (self.cr3 & self.bit_not(0b111, 3)) | esm
        self.writeReg(self.REG['CR0'], self.cr0)
        self.writeReg(self.REG['CR3'], self.cr3)



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
T = 1e-3
qw = 5
qb = 0.01
r_theta = 5e-5
rw = 1e-6
Ad = np.array([[1, 0, 0], [T, 1, 0], [0, 0, 1]])
q_01 = (T**2) * qw / 2
Qd = np.array([[T * qw, q_01, 0], [q_01, (T**3) * qw / 3, 0], [0, 0, T * qb]])
C = np.array([[0, 1, 0], [1, 0, 1]])
I = np.identity(3)
R = np.array([[r_theta, 0], [0, rw]])
rad_deg = np.pi / 180


# kalman filter functions
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



# IMU related functions
def MPU_Init():
    # write to Register 25 Sample Rate Divider
    # Sample_rate=gyro_out_rate/(1+SMPLRT_DIV) & gyro_out_rate = 8khz if CONFIG = xxxxx-000 or 111
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)    
    
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


def read_new_value():
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    gyro_x = read_raw_data(GYRO_XOUT_H)
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    Gx = gyro_x/131.0
    return Ay, Az, Gx



# control allocation functions
def torque_to_w(torque):
    slope = -0.048         # slope = (y2 - y1)/(x2 - x1) torque on y-axis, w on x-axis
    if torque > 0.7:
        torque = 0.7       # if the controller asks for more torque than the motor can 
    # NOTE: need some way to make sure now phi_dot is always positive after abs()
    # DOUBT: does phi_dot value really matter?
    # stall_torque = 0.8     # change stall torque according to your motor specification
    # if torque < -0.7:
    #    torque = -0.7
    w = (torque/slope) + 16.67
    return w


def controller(ctrl_input):
    K = np.array([-1.4142, 1.0848, 0.2093, 0.6905])
    y = -(K[0]*ctrl_input[0] + K[1]*ctrl_input[1] + K[2]*ctrl_input[2] + K[3]*ctrl_input[3])
    # needs to be updated to add A, B, C matrices. Currently y = -Kz
    return y



def step_forward(step_count, delay):
    if step_count < 10:
        step_count = 10
    print("start forward", step_count)
    GPIO.output(DIR, GPIO.HIGH)
    for i in range(int(step_count)):
        print("f", i)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

        
def step_backward(step_count, delay):
    if step_count < 10:
        step_count = 10
    print("start back", step_count)
    GPIO.output(DIR, GPIO.LOW)
    for i in range(int(step_count)):
        print("b", i)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

        
# initializing the stepper motor driver and motor parameters
maxCurrent = 1800        # unit miliamps. change according to your motor specifications
steps_per_rev = 200      # total steps for 1 rev for the motor. change according to your motor specifications
u_steps = 16             # we set the microstepping mode to 16

stepper1 = AMIS30543()
stepper1.initSS(SlaveSelect)
GPIO.output(STEP, GPIO.LOW)
GPIO.output(DIR, GPIO.LOW)
sleep(1);     #Give the driver some time to power up.
stepper1.reset_settings()    
stepper1.setCurrentMilliamps(maxCurrent);
stepper1.setStepMode(u_steps)
stepper1.enableDriver()






bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

# initializing states for first run
state_arr = np.array([0, 0, 0, 0], dtype='float64')     # states = phi, theta, phi_dot, theta_dot
t = 1e-3    # the update time for multilying with phi_dot

# initialization for the kalman filter
MPU_Init()
mu, P = initialise_kf()

# to refcord the output for future debugging
tau_array = np.array([0])
states = np.array([0])
theta_array = np.array([0])



for i in range(100000):
    tau = controller(state_arr)
    phi_dot = torque_to_w(abs(tau))
#    print("new tau and phi_dot", tau, phi_dot)
    phi = phi_dot * t
    np.append(tau_array, tau)
    
    
    freq = phi_dot * steps_per_rev * u_steps
    delay = (1/freq)/2
    
    if tau < 0:
        step_backward(phi, delay)
        
    elif tau >= 0:
        step_forward(phi, delay)
    
    Ay, Az, Gx = read_new_value()
    y_theta = np.arctan2(Az, Ay)                  # y_theta output in radians
    yw = Gx * rad_deg                             # output in rad/sec

    mu, P = time_update_kf(mu, P)
    y = np.array([y_theta, yw])
    y.shape = (2, 1)
    mu, P = measurement_correction(mu, P, y)
    theta_dot = (mu[0] + mu[2]) / rad_deg           # theta_dot output in rad/sec
    theta = (mu[1] / rad_deg) + 90                  # theta output in deg
    
#    print("new theta", theta)
    
    np.append(states, state_arr)
    np.append(theta_array, theta)
    state_arr[0] = phi
    state_arr[1] = theta
    state_arr[2] = phi_dot
    state_arr[3] = theta_dot
#    if phi_dot < 0:
#        print("phi change motor direction")
#    if tau < 0:
#        print("tau change motor direction")
        




stepper1.close_port()

GPIO.cleanup()
