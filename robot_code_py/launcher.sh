#!/bin/sh
# launcher.sh

sudo pigpiod -s 1
cd /
cd home/pi/research-internsip-sbr/robot_code_py
sudo python3 sb_robot_integrated.py
cd /
