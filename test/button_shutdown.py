# code from https://www.quartoknows.com/page/raspberry-pi-shutdown-button
# !/bin/python
# Simple script for shutting down the Raspberry Pi at the press of a button.
# by Inderpreet Singh

import RPi.GPIO as GPIO
import time
import os

 
button = 37
# Use the Broadcom SOC Pin numbers

# Setup the pin with internal pullups enabled and pin in reading mode.

GPIO.setmode(GPIO.BOARD)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
 

# Our function on what to do when the button is pressed
def Shutdown(channel):
    print("Shutting Down")
    time.sleep(5)
    # os.system("sudo shutdown -h now")
 

# Add our function to execute when the button pressed event happens
GPIO.add_event_detect(button, GPIO.FALLING, callback=Shutdown, bouncetime=2000)

# Now wait!
while 1:
    time.sleep(1)
