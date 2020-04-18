import pigpio
import RPi.GPIO as GPIO
import AMIS30543 as amis

# defining a new SS because we are not able to control the actual SS pin with spidev library
# connect the ss pin of slave to the below pin and not to SPI0 CE1
SlaveSelect = 33
DIR = 31
STEP = 29

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SlaveSelect, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setwarnings(False)

stepper1 = amis.AMIS30543(0, 1)

stepper1.initSS(SlaveSelect)
GPIO.output(STEP, GPIO.LOW)
GPIO.output(DIR, GPIO.LOW)
sleep(1);                   #Giving the driver some time to power up.

stepper1.reset_settings()   # possibly redundant to call the function here. apply the settings when constructor is called

maxCurrent = 1800
stepper1.setCurrentMilliamps(maxCurrent)

total_steps = 200           # number of steps for the motor to complete one revolution. found by 360/step_angle
micro_steps = 16
stepper1.setStepMode(micro_steps)
stepper1.enableDriver()

rev_per_sec = 2
one_rev = total_steps * micro_steps # number of steps req. for one entire revolution of the wheel

# x rev/sec => x*3200 steps/sec 
# time period = 1/(x*3200)
# so, frequency = x*one_rev
PWM_freq = one_rev * rev_per_sec 
pwm_obj = pigpio.pi()

GPIO.output(DIR, GPIO.HIGH)

pwm_obj.hardware_PWM(STEP, PWM_freq, 5000000)

input("Enter any letter to exit: ")

pwm_obj.hardware_PWM(STEP, 0, 0)    # motor keeps on spinning even after ports closed, if the freq & duty cycle not 0
stepper1.close_port()

GPIO.cleanup()
