"""
use this code with 'sudo pigpiod -s 1' i.e. start with sample rate 1 microsecond
and view with 'piscope &'
visit abyz.me.uk/rpi/pigpiod
"""
import pigpio
from time import sleep

# it is not required to set mode of GPIO pins.
# pigpio sets the pins in BCM mode by default.
STEP = 18

pwm_obj = pigpio.pi()

freq = 1000

pwm_obj.hardware_PWM(18, freq*2, 500000)
sleep(2)
pwm_obj.hardware_PWM(18, freq, 500000)
sleep(2)
pwm_obj.hardware_PWM(18, 1000, 500000)

pwm_obj.set_PWM_frequency(18, freq*2)
pwm_obj.stop
pwm_obj.set_PWM_frequency(18, freq)
input("Enter the loop (notice pin {} in piscope)".format(STEP))
for i in range(100000):
    if i % 10000 == 0:
        sleep(2)
        print(freq)
        freq += 1000
        pwm_obj.hardware_PWM(18, freq, 500000)

input("Enter to stop the process")
pwm_obj.hardware_PWM(STEP, 0, 0)
pwm_obj.stop()
