import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from time import sleep        


SlaveSelect = 33
DIR = 31
STEP = 29


# motor driver class containing functions

class AMIS30543:
    def __init__(self):
        self.wr = 0
        self.cr0 = 0 
        self.cr1 = 0 
        self.cr2 = 0 
        self.cr3 = 0
        self.spi = spidev.SpiDev()    # Enable SPI
        bus, device = 0, 1    #Device is the chip select pin. Set to 0 or 1, depending on the connections
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
        self.cr0 = (self.cr0 & 0b11100000) | CUR_reg;
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
            
        self.cr0 = (self.cr0 & self.bit_not(0b11100000, 8)) | (sm << 5)
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


    w = (torque/slope) + 16.67    # this value is in rev/sec we also need rad/sec for controller
    return w


def controller(ctrl_input):
    # K = np.array([-1.4142, 1.0848, 0.2093, 0.6905])
    K = np.array([-2.2361, 0.7272, 0.1238, 0.4621])
    y = -(K[0]*ctrl_input[0] + K[1]*ctrl_input[1] + K[2]*ctrl_input[2] + K[3]*ctrl_input[3])
    # needs to be updated to add A, B, C matrices. Currently y = -Kz
    return y



def step_forward(step_count, delay):
    if step_count < 10:
        step_count = 50
    # print("Go forward {} steps".format(step_count))
#     GPIO.output(DIR, GPIO.HIGH)
#     for i in range(int(step_count)):
#         print("f", i)
#         GPIO.output(STEP, GPIO.HIGH)
#         sleep(delay)
#         GPIO.output(STEP, GPIO.LOW)
#         sleep(delay)

        
def step_backward(step_count, delay):
    if step_count < 10:
        step_count = 50
    # print("Go back {} steps".format(step_count))
#     GPIO.output(DIR, GPIO.LOW)
#     for i in range(int(step_count)):
#         print("b", i)
#         GPIO.output(STEP, GPIO.HIGH)
#         sleep(delay)
#         GPIO.output(STEP, GPIO.LOW)
#         sleep(delay)

        
# initializing the stepper motor driver and motor parameters
maxCurrent = 1800        # unit miliamps. change according to your motor specifications
steps_per_rev = 200      # total steps for 1 rev for the motor. change according to your motor specifications
u_steps = 16             # we set the microstepping mode to 16








# bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

# initializing states for first run
state_arr = np.array([0, 0, 0, 0], dtype='float64')     # states = phi, theta, phi_dot, theta_dot
t = 1e-3    # the update time for multilying with phi_dot

# initialization for the kalman filter
#MPU_Init()
mu, P = initialise_kf()

# to refcord the output for future debugging
# tau_array = np.array([0])
tau_array = []
states = np.array([0])
theta_array = np.array([0])





df = pd.read_csv('example1.csv')
df2 = df.drop(['Time (sec)', 'Voltage (V)', 'Gyro45(deg/sec)', 'Theta (deg)'], axis=1)
# df3 = df[:1000].copy()
accX = np.array(df['AccX (m/s^2)'].tolist())
accY = np.array(df['AccY (m/s^2)'].tolist())
gyro = np.array(df['Gyro (deg/sec)'].tolist())


y_theta = np.arctan2(-accY, accX)               # y_theta output in radians
yw = gyro * rad_deg                             # output in rad/sec

mu_dtheta = []                                        # empty list to record values of mu_w + mu_b
mu_theta = []                                         # empty list to record values of mu_theta
phi_tp = []
dphi_tp = []




for i in range(1000):
    tau = controller(state_arr)
    phi_dot = torque_to_w(abs(tau))
    phi = phi_dot * t
    tau_array.append(tau)
    phi_rad = phi * 2 * np.pi
    phi_dot_rad = phi_dot * 2 * np.pi
    phi_tp.append(phi_rad)
    dphi_tp.append(phi_dot_rad)
    # print("Torque = {}Nm, phi_dot = {}rad/sec, phi = {}rad".format(tau, phi_dot_rad, phi_rad))
    #print("".format())
    
    
    freq = phi_dot * steps_per_rev * u_steps
    delay = (1/freq)/2
    
    if tau < 0:
        step_backward(phi, delay)
    elif tau > 0:
        step_forward(phi, delay)
    

    mu, P = time_update_kf(mu, P)
    y = np.array([y_theta[i], yw[i]])
    y.shape = (2, 1)
    mu, P = measurement_correction(mu, P, y)
    theta_dot = (mu[0] + mu[2])                     # theta_dot output is in rad/sec because states are in radians
    theta = mu[1] + (np.pi/2)                        # theta output in radians. corrected by adding pi/2
    mu_theta.append(theta)
    mu_dtheta.append(theta_dot)
    
    # We update the state values in this section
    np.append(states, state_arr)
    np.append(theta_array, theta)
    state_arr[0] = phi_rad               # multiply by 2*pi for phi rev -> phi rads
    state_arr[1] = theta
    state_arr[2] = phi_dot_rad          # multiply by 2*pi for rev/sec -> rad/sec



nptorque = np.array(tau_array)
npdtheta = np.array(mu_dtheta)
nptheta = np.array(mu_theta)
npdphi = np.array(dphi_tp)
npphi = np.array(phi_tp)
x_axis = np.arange(200)


plt.figure(1)
plt.plot(nptheta, label="theta_estimate")
plt.ylabel('theta estimate')
plt.figure(2)
plt.plot(npdtheta, label="dtheta_estimate")
plt.ylabel('dtheta estimate')
"""
plt.figure(3)
plt.plot(npdphi, label="dphi_estimate")
plt.ylabel('dphi estimate')
plt.figure(4)
plt.plot(npphi, label="phi")
plt.ylabel('phi estimate')
plt.figure(5)
plt.plot(nptorque, label="torque_estimate")
plt.ylabel('torque estimate')
"""




plt.show()

# stepper1.close_port()

# GPIO.cleanup()
"""
plt.subplot(4, 1, 1)
plt.plot(x_axis, nptheta)
plt.title('states and torque')
plt.ylabel('theta estimate')

plt.subplot(4, 1, 2)
plt.plot(x_axis, npdtheta)
plt.ylabel('dtheta estimate')


plt.subplot(4, 1, 3)
plt.plot(x_axis, npdphi)
plt.ylabel('dtheta estimate')


plt.subplot(4, 1, 4)
plt.plot(x_axis, npphi)
plt.ylabel('dtheta estimate')
"""
