from time import sleep
import RPi.GPIO as GPIO
#GPIO.cleanup()
#GPIO.cleanup()
DIR = 16    # Direction GPIO Pin
STEP = 18   # Step GPIO Pin
CW = 1      # Clockwise Rotation
CCW = 0     # Counterclockwise Rotation
total_steps = 200   # Steps (360 / 1.8)
MODE = (40, 38, 36)   # Microstep Resolution GPIO Pins
u_steps = 16              #number of microstepe per step
RESOLUTION = {1: (0, 0, 0),        #FULL
              2: (1, 0, 0),
              4: (0, 1, 0),
              8: ( 1, 1, 0),
              16: (1, 1, 1)}        #1/16


GPIO.setmode(GPIO.BOARD)
#GPIO.setwarnings(False)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.output(DIR, CW)
GPIO.setup(MODE, GPIO.OUT)
GPIO.output(MODE, RESOLUTION[u_steps])

rev_per_sec = 50
step_count = total_steps * u_steps * rev_per_sec
delay = (1 / step_count) / 2
n=0


for x in range(step_count):
#while True:
    
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)
#    n+=1
#    if n%15000==0:
#        break
#        print(rev_per_sec)
#        rev_per_sec += 1 
#        step_count = total_steps * u_steps * rev_per_sec
#        delay = (1 / step_count) / 2
#        if rev_per_sec==16:
#            break
#sleep(0.5)
#GPIO.output(DIR, CCW)
#for x in range(step_count):
#    GPIO.output(STEP, GPIO.HIGH)
#    sleep(delay)
#    GPIO.output(STEP, GPIO.LOW)
#    sleep(delay)
GPIO.cleanup()
