#!/bin/sh
# launcher.sh

sudo pigpiod -s 1
cd /
cd home/pi/sbr_rise
sudo python3 button_final_all.py
cd /
