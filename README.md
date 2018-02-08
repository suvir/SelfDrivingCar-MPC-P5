# CarND-Control-MPC

This repository contains all the code needed to complete the MPC project for Udacity's Self-Driving Car Nanodegree.

---
## Submission
Files modified
- MPC.cpp
- MPC.h
- main.cpp
---

# Writeup

## Model

This project implements the Model Predictive Control (MPC) to safely drive an autonomous car around the track.

### State Vector
The state of the car at any given time is modelled by the six variables below:
* `x`: x coordinate of the vehicle.
* `y`: y coordinate of the vehicle.
* `psi`: yaw angle of the vehicle.
* `v`: velocity of the vehicle.
* `cte`: cross track error at this moment.
* `epsi`: orientation error

### Actuators
The state of the car can be changed by changing the steering angle, pressing gas or braking. These are captured in the actuator actions:
* `delta`: steering wheel angle. This is restricted to (-25,25) degrees.
* `a`: throttle. This is restricted to (-1,1)

### Update equations
The MPC model prescribes the following equations for updates to the vehicle's state
* `x = x + v * cos(psi) * dt`
* `y = y + v * sin(psi) * dt`
* `psi = psi + v / Lf * delta * dt`
* `v = v + a * dt`
* `cte = cte + v * sin(epsi) * dt =  f(x) - y + v * sin(epsi) * dt`
* `epsi = epsi + v / Lf * delta * dt = psi - psi_des + v / Lf * delta * dt`

In the above equations, `psi_des` is the desired `psi` or yaw angle.

### Other constants
* `Lf`: Length from front of vehicle to the centre of gravity.

## Timestamp length and duration

`T` is the duration for which the trajectory will be predicted. This value needs to finely tuned.

In the lectures, T was recommended to be of the order of a few seconds.

`T` is set by tuning the timestamp length `dt` and number of discretization `N`.

`T = N*dt` where,
* `N` is the number of discretizations. A large value will be computationally taxing and a low value will result in low accuracy.
* `dt` is the discretization of `T`.

### Chosen values
For the submission, I chose `N` = 10 and `dt` = 0.1.
I tried larger values of `N` and `dt`. This typically resulted in the trajectory exceeding the horizon and causing errors. While the car did stay on track for the most part, it would drive off the track on some sharp turns. Also, this is computationally more intensive.

For smaller values of `N` and `dt`, the accuracy was much lower. The car would almost always drive off the center of the road and eventually start driving outside driveable area.

## Polynomial fitting and MPC preprocessing
Polynomial was fit using the provided `polyfit` function.

Preprocessing was done for `x, y` and `psi` prior to the MPC procedure. They were all set to zero.

* This helps transform the global coordinates of the waypoints into the view of the vehicle.
* Preprocessing should also simplify the numerical calculation for `cte` and `psi`.

## MPC with latency
In a real-world scenario, there is a gap between applying actuation and it actually taking effect. A value of 100 ms is considered reasonable in the industry. Control models need to account for this latency.

`MPC` is particularly suited to handling latency since it can be part of the model itself.

To account for latency in my implementation:
* A 100 ms lag was introduced between changes to actuators and recalculation of state vector.
* The coordinates of waypoints would be changed as per this new state after 100 ms.
* Other variables, like `cte`, `epsi` and `coeffs` would also be based off state after the 100 ms delay.