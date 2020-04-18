# Self-balancing roobot - Research Internship

This project entails the mathematical modelling, design and simulation of a two-wheeled self-balancing robot followed by the hardware implementation on an embedded system. This project was performed in the practical course Research Internship in Systems Engineering offered in the University of Passau in SS19.
The supervisors of this course were Prof. Dr. Fabian Wirth and Ms. Roxanne Jackson.
The student participators in this project were:
Harish Gopal			swamin01@ads.uni-passau.de
Ram Prasanth			udhaya01@ads.uni-passau.de
Rutvik Bhatt			bhatt02@ads.uni-passau.de
Shrinivas Iyengar		iyenga01@ads.uni-passau.de

Linear Quadratic Regulator (LQR) is implemented to control the linearized dynamic system. Kalman filter is used for state estimation.

## Getting started
This repository contains the Python, MATLAB and Simulink files for this project. The mathematical model and LQR controller were tested on MATLAB and Simulink. The final code was implemented in Python on a Raspberry Pi. 

## Main robot code
At first we describe what runs the "self-balancing" part of the robot. 

this file depends on many files which will also be described shortyly

sb_robot_integrated.py 
