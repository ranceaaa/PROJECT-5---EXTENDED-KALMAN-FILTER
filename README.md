# Extended Kalman Filter Project

The goals / steps of this project are the following:
* Build a Kalman Filter
* Use first measurements to initialize the state vectors and covariance matrices
* Predict and then update the prediction
* Use Radar and Lidar sensor data

[//]: # (Image References)

[image1]: ./images/dataset1.png "Dataset1"
[image2]: ./images/dataset2.png "Dataset1"

---

## Prerequisites

* Term 2 Simulator with uWebSocketIO
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

If you want to install the libraries on linux Operating System use install-linux.sh(./install-linux.sh) or for OSX use install-mac.sh(./install-mac.sh).

---

## Basic Build Instructions

1. Clone this repo.
2. Select build directory: `cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./ExtendedKF `

## Your code should compile

---

The code compiles without errors with cmake and make. CMakeLists.txt(./CMakeLists.txt) is the one from the seed project.

---

## Root-mean-square deviation (RMSE) 

#### Requirement : px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52]

The results for both datasets can be seen below. 

![alt text][image1]
![alt text][image2]

Hurray !!!!. For both datasets the results are lower than the required values.

---

## Follow the Correct Algorithm

### Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

Kalman Filter algorithm can be found in kalman_filter.cpp(./kalman_filter.cpp)

###  Use the first measurements to initialize the state vectors and covariance matrices.

The initialization can be seen from line 67 to 121 in FusionEKF.cpp(./FusionEKF.cpp)

### Kalman Filter algorithm first predicts then updates.

Prediction can be found on line 145 and the updates from line 153 to 163 in FusionEKF.cpp(./FusionEKF.cpp)

### Kalman Filter uses radar and lidar measurements.

Radar and lidar measurements are both used in the initialization step( from line 72 to 108) and in the update step (from line 153 to 163) in the FussionEKF.cpp(./FusionEKF.cpp) file. 


## PLEASE PAY ATTENTION TO THE FORMULAS.
