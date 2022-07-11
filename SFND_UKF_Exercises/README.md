# Unscented Kalman Filter Exercises

## Requirements

My specifications are as follows, 

* Xubuntu 18.04
* CMake v2.23.2
* Eigen C++ v3.3.4
* C++ v11
* gcc v7.5.0

## Exercises

Below exercises incrementally implements Unscented Kalman Filter.

### 1. [UKF_1](./UKF_1/)

Estimate sigma points in input space (5D).

### 2. [UKF_2](./UKF_2/)

Augment the input space to account for acceleratation noise and yaw rate noise (7D). Estimate the sigma points in the augmented space as done in previous exercise.

### 3. [UKF_3](./UKF_3/)

Apply the non linear transformation on above calculated sigma points.

### 4. [UKF_4](./UKF_4/)

Update state mean and covariance matrix given the output of previous step.

### 5. [UKF_5](./UKF_5/)

Given the predicted sigma points, transform each into the measurement space (**Radar**). Calculate the mean predicted measurement and the innovation covariance matrix given the Radar's measurement noise parameters. 

### 6. [UKF_6](./UKF_6/)

Update the state mean and covariance matrix according to UKF equations using the calculated parameters from the previous step.




## Usage

Go to respective directory attached with above links after cloning this repo,

```bash
mkdir build && cd build
cmake ..
make
```

Running above commands will generate the executable.

