"""
Code originally from
https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi

for more information about register address and map visit
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
"""

"""
important information about acc and gyro refresh rate:
LQR controller is synced with the refresh rate of imu.
acc is always at 1 kHz. gyro output rate is at 8 kHz 
if DLPF in CONFIG is either 0 or 7 and 1 kHz otherwise.
"""
import smbus

# MPU6050 register addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19                           # sample rate divider = gyro_rate / (1 + SMPLRT_DIV_value)
CONFIG = 0x1A                               # | - - | EXT_SYNC[2:0] | DLPF[2:0] |
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1)                        # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68                       # MPU6050 device address


def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if value > 32768:
        value = value - 65536
    return value


if "__name__" == "__main__":
    MPU_Init()

    while True:
        # Read Accelerometer raw value
        print("Reading Data of Gyroscope and Accelerometer")
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0

        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0

        print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s",
              "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)
