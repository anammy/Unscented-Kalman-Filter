# Unscented Kalman Filter Project

The objective of this project is to implement an unscented kalman filter to estimate the state of a moving vehicle with noisy lidar and radar measurements. The performance of the kalman filter was evaluated by calculating the root mean squared error, RMSE, over the track and ensuring it is lower than the tolerance.

[//]: # (Image References)
[image1]: ./pictures/Dataset1.png
[image2]: ./pictures/Dataset2.png

This project requires the following files to run:
* Utiltiies: Run script 'install-ubuntu.sh' for Linux, 'install-mac.sh' for Mac, 'install-ubuntu.sh' in Ubuntu Bash 16.04 for Windows.
 * cmake: 3.5
 * make: 4.1 (Linux and Mac), 3.81 (Windows)
 * gcc/g++: 5.4
 * [uWebSocketIO](https://github.com/uNetworking/uWebSockets)
* Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

The starter code from [Udacity](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) was used. The code in the following files were completed to implement the UKF: src/ukf.cpp, src/ukf.h, src/tools.cpp, and src/tools.h. The program main.cpp uses uWebSocketIO to communicate with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Tuned Parameters

The initial estimates for the state vector `x` and the covariance matrix `P` were manipulated for the two scenarios/datasets. The initial estimates for `x` was `[px, py, 1.0, 1.0, 0.09]` and `[px, py, 1.0, 3.0, 0.09]` for datasets 1 and 2, respectively. px and py in the initial state vector were position values from the first lidar or radar measurement for the respective dataset. A low speed of 1 m/s was chosen as the initial starting velocity of the car. The initial yaw was chosen to be approximately 60 deg (1 rad) and 170 deg (3 rad) for datasets 1 and 2, respectively. A small yaw rate of 5 deg/s (0.09 rad/s) was chosen for both datasets since the car was not expected to take sharp turns at the beginning of the track. The initial covariance matrix `P` was chosen to be a diagonal matrix with the following values on its main diagonal `[0.15, 0.15, 1, 1, 1]`. The variances of px and py were chosen to be 0.15 in the initial `P` since the initial estimates for px and py would be extracted from the first lidar or radar measurement. Since the standard deviations of the measurement noise from the sensor manufacturer for both lidar and radar are lesser than 1, it is estimated that the initial standard deviation of px and py would be lesser than 1.

The other parameters to be tuned include the process noise standard deviations for the linear and angular accelerations. These parameters were tuned (i.e. significantly lowered) roughly at first to crudely follow the car along the track and lower the RMSE. Then, the Normalized Innovation Squared (NIS) value was used to fine-tune the process noise standard deviations by calculating the percentage of NIS values that were greater than 7.82 and 5.99 for radar and lidar, respectively. NIS values follow the Chi-squared distribution and so, the standard deviations were tuned to give a percentage close to 5% (0.05 from the Chi-squared distribution) for 3 DOF (radar) and 2 DOF (lidar) measurement spaces. A set of standard deviations were determined using the NIS values and were then evaluated to see which pair of values gives a lower RMSE. The linear and angular acceleration standard deviations were chosen to be 1.0 and 0.3, respectively.

## UKF Results

The RMSE for the vehicle position (px, py) and velocity (vx, vy) calculated over the track for dataset 1 and 2 satisfied the following tolerance:
* px <= 0.09
* py <= 0.10
* vx <= 0.40
* vy <= 0.30

The following is a snapshot of the final simulator result for datasets 1 and 2:

![Result-Dataset1][image1]

![Result-Dataset2][image2]

The RMSE values from the UKF are lower than those from the Extended Kalman Filter (EKF) project, especially the velocity RMSE, because the CTRV model is more precise than the constant velocity model used in the EKF project. In addition, the UKF is known for handling non-linear equations through the calculation of sigma points better than the linearization performed in the EKF.
