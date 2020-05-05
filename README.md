# Self-balancing robot - Research Internship

This project entails the mathematical modelling, design and simulation of a two-wheeled self-balancing robot followed by the hardware implementation on an embedded system. This project was performed in the practical course Research Internship in Systems Engineering offered in the University of Passau in the Summer Semester 2019.
The supervisors of this course were Prof. Dr. Fabian Wirth and Ms. Roxanne Jackson.
The student participators in this project were:  

Harish Gopal  
swamin01@ads.uni-passau.de
harishgopal1996@gmail.com 

Ram Prasanth  
udhaya01@ads.uni-passau.de
ramprasanth.u@gmail.com 

Rutvik Bhatt  
bhatt02@ads.uni-passau.de
bhatt12@gmail.com

Shrinivas Iyengar  
iyenga01@ads.uni-passau.de
shrinivas.iyengar10@gmail.com

### Getting started
Linear Quadratic Regulator (LQR) is implemented to control the linearized dynamic system. Kalman filter is used for state estimation.
This repository contains the Python, MATLAB and Simulink files for this project. The mathematical model and the LQR controller were tested on MATLAB and Simulink. The final code was implemented in Python on a Raspberry Pi. Below is a description of some of the components used for the physical structure. More details can be found in the [project report](robot_code_py\Research-Internship-report.pdf).
1. Two NEMA-17 stepper motors (each with rated torque = 0.8 Nm) were used in this project. The required rated current per phase is 1.8 A. Thus the appropriate motor driver was one with an SPI interface, the [Polulu AMIS-305433](https://www.pololu.com/product/2970). 
2. An MPU-6050 is used for accelerometer and gyroscope measurements. 

### Main robot code

The main program to run on the Raspberry Pi is named [sb_robot_integrated](robot_code_py\sb_robot_integrated.py). This code mainly depends on the [motor driver](robot_code_py\AMIS30543.py), the [Kalman Filter](robot_code_py\kalman_filter.py), and the [imu](robot_code_py\mpu6050.py) code, amongst others. The `controller()` function is where the "self-balancing" part of the robot is implemented. The value of the feedback matrix *K* is obtained from the [MATLAB code](matlab_simulink\controller.m) and placed in this function. Here we describe the essential parts needed replicate this project:
1. The [motor driver](robot_code_py\AMIS30543.py) code contains very specific interfacing options between the AMIS-30543 and the Raspberry Pi. Specifically, we only set the current and the micro-step mode. Both of these values can be set by sending specific values to the respective registers. More possible configurations can be added by referring to Table 12 and Table 13 of the AMIS-30543 datasheet. The motor driver offers many features, which were not added simply because they were not needed for the project. 
    1. We test the interfacing between these two devices by running the motor. In the [first case](robot_code_py\stepper_simple_delay.py) we provide the PWM signal by a simple delay function `time.sleep()`. 
    2. The `sleep()` function is not suitable for use in embedded systems. Aditionally, it is also not adequate for the precise PWM signals, with time period in the order of a few hundread micro-seconds, that is required to run the robot. So, we switch to a library that is able to provide such functionality, the [pigpio](http://abyz.me.uk/rpi/pigpio/). To test this library, the [pigpio pwm](robot_code_py\stepper_pigpio_pwm.py) code runs the motor at a specific speed withc accurate PWM signals. 
        1. To use the pigpio library, it needs to be installed first, with instructions provided in the homepage. There are two example programs in this repository to understand the main PWM function. Apart from the library, the above website contains instructions to install and use [piscope](http://abyz.me.uk/rpi/pigpio/piscope.html), a virtual oscilloscope. Though it is not necessary for functionality, we recommend it as it helps during testing and is easy to use. 

2. The [Kalman Filter](robot_code_py\kalman_filter.py) code can estimate the angle of tilt and the angular velocity of the IMU. First, call the `initialise_kf()` method to initialise the mean estimate and the covariance matrix. Then, use the `time_update_kf()` and `measurement_correction()` methods iteratively, to implement the Prediction and Correction steps respectively. Both the functions require, as arguments the mean and covariance from previous time steps. Additionally, `measurement_correction()` requires the measurements from accelerometer and gyroscope as an input. In
    1. The Kalman Filter is [validated](robot_code_py\kf_csv_validation.py) using measurements from an unbaised gyroscope and encoder, stored in a [csv file](robot_code_py\kf_data_validation.csv). 
    2. After successfully validating the csv data, we implement the Kalman Filter on live [imu data](robot_code_py\kf_imu_live.py). 

3. Finally, the [linearized_matrices](robot_code_py\linearized_matrices.py) file contains the linear matrices described in the project report. They are used in estimating the wheel angle and angular velocity parameters, as described in Section 4.2 of the report. 

### Simulations using Matlab and Simulink
The non-linear model derived in Section 2.3 of the project report has been tested via the MATLAB [code](matlab_simulink\nl_diff_eqn.m) and a Simulink [model](matlab_simulink\non_linear_v2.slxc). Both of these files plot the behaviour of the tilt angle, `theta` and the required torque input for the plant. Both of these files depend on `A_matrix`, `B_matrix`, `C_matrix` and `damping_matrix`, which, together, implement Equation (7) in Section 2.3 of the report. 

Despite their names, they are not the state-transition, contol or measurement matrices. The linearized state and control matrices are implemented [separately](matlab_simulink\linear_matrices.m). To validate the controller, change the *Q* and *R* values to obtain the feedback matrix *K*. As *K* is saved in the workspace, it can be directly used in the Simulink model and MATLAB non-linear code. However, to use it to stabilise the physical robot, the value needs to be copied into the [python code](robot_code_py\sb_robot_integrated.py), as described above.


As of December 2019, the project still lacs some functionality, for easier use and accesibility, which could not be added due to time constraints. For doubts regarding the project please feel free to email the student participants. Personal email-id's have been provided in case the university id's are no longer reachable.