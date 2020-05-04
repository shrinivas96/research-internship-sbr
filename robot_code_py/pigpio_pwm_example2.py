"""
use this code with 'sudo pigpiod -s 1' i.e. start with sample rate 1 microsecond
and view with 'piscope &' 
visit abyz.me.uk/rpi/pigpiod
"""
import pigpio

# it is not required to set mode of GPIO pins.
# pigpio sets the pins in BCM mode by default.
STEP = 18 
pwm_obj = pigpio.pi()

# hardware_PWM(pin, freq, duty_cycle) duty_cycle = 500000 for 50%
while True:
    i = input("enter desired frequency (n to exit):")
    if i == 'n':
        pwm_obj.hardware_PWM(STEP, 0, 0) # setting freq and duty_cycle to 0 to stop
        pwm_obj.stop()
        break
    if int(i) < 0:
        pwm_obj.write(23, 0)
    else:
        pwm_obj.write(23, 1)
    pwm_obj.hardware_PWM(STEP, abs(int(i)), 500000)