# Vehicle Detection and Tracking
> Please refer to: Self-Driving Car Engineer Nanodegree - Udacity, [Term 2 - Project 5](https://github.com/udacity/CarND-MPC-Project)

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


### Overview
This project aims to apply a predictive control to autonomously drive a car on a track using a kinematic model.

### Implementation

* Clone or download the entire project from [here](https://github.com/udacity/CarND-MPC-Project). 
* Replace two of the original files with my solutions:
    - 
    -  
* Follow the implemtation instruction described [here](https://github.com/udacity/CarND-MPC-Project/blob/master/README.md).  

This [video](https://vimeo.com/223588171) shows the outcome of the project.  


# Model Predictive Control (MPC) Project

---
## Implementation notes 
---
### The Model  

> _Student describes their model in detail. This includes the state, actuators and update equations._  


In this project, I apply a predictive control to autonomously drive a car on a track via a kinematic model. Kinematic models are a subset of more complex and complete "dynamic models", and share with them the approach of optimizing on a finite time horizon while predicting and then implementing a solution for a shorter period. In particular, for this characteristic, both differ from the proportional–integral–derivative controller (PID controller) models. 

The pipeline for building up an MPC can be summarized as follows:

1. Define a set of variable to design the "state" of the car in the model
   * In this project, the state is characterized by 4 variables [ x,y,ψ,v ], where:  
        - x and y are the coordinates (position) to identify where the vehicle is located
        - ψ is the orientation of the vehicles
        - v is the velocity of the car (the dynamic component of the model)
2. Compare the state (x,y) to a map, to determine where the vehicle is (the environmental step)
3. To control the entire process, include a (small) set of variable, the so-called actuators, considering that inputs from actuators determine the changes in the state of the vehicle. 
    * In this project, I use two actuators: [ δ, a ]
        - δ is the steering wheel 
        - a is the throttle pedal (acceleration), that, when negative, operate for braking   
4. Take also into account external parameter. E.g.:
     * Lf, the distance between the front of the vehicle and its center of gravity (larger values slow the turn rate)
     * the time interval between the input (of actuators) and their effect; this latency should be carefully managed when (in this project I apply a delay in activating actuators)
5. Model the evolution of the model and how the actuators can allow the state to evolve over a finite time interval (dt):
    * x(t+dt) = x(t) + v(t) ∗ cos(ψ(t)) ∗ dt 
    * y(t+dt) = y(t) + v(t) ∗ sin(ψ(t)) ∗ dt 
    * ψ(t+dt) = ψ(t) + (v(t)/Lf) ∗ δ(t) ∗ dt 
    * v(t+dt) = v(t) + a(t) ∗ dt
5. Choice a trajectory (the reference trajectory), using the environmental model, the map, and the vehicle location. In this so-called planning step, define the reference trajectory by  fitting a polynomial among given waypoints 
6. Eventually, constrain actuators in some way. In this project, I set:
    * some limits to the steering angle (therefore indirectly limiting ψ) 
    * a reference positive velocity, to allow the car to move even when centered over a target point (actually, this will operate on the acceleration via the throttle)
7. Fix a target function, for one or more variables, to optimize the process by adjusting the actuators. In this step, you might want to minimize two variables:  
    * cte, the absolute value of the "cross track error" between the reference trajectory and the vehicle’s actual path, taking into account the cte's dynamic:
        - cte(t+dt) = cte(t) + v(t) ∗ sin(eψ(t)) * dt    
    * eψ, the difference/error between the vehicle orientation and the trajectory orientation. Once calculated the desired orientation (ψdes) from the reference trajectory, eψ can be expressed taking considering that:
        - eψ(t) = ψ(t) - ψdes(t)
        - eψ(t+dt) = eψ(t) + (v(t)/Lf) ∗ δ(t) ∗ dt
        
        
### Timestep Length and Elapsed Duration (N & dt)  

> _Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally, the student details the previous values tried._  

In the [study](http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf) referenced in the lessons, Kong et al. show that a kinematic model discretized at 200 ms works as well as a dynamic model discretized at 100 ms and better than a kinematic model discretized at 100 ms. Keeping this results as the initial reference for setting the elapsed duration between timesteps, I tried different values of N to obtain a total prediction interval between 2 and 5 seconds (i.e. 10<=N<=25). After some attempts (that also involved the dt value), I improved my results once I changed the relative weight to the components of my cost function. In the end, the hints in the study were confirmed and I landed on a 3 sec. total prediction time, with dt=0.2 (200 ms) and N=15. 

### Polynomial Fitting and MPC Preprocessing 

> _A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described._  

Before fitting a polynomial among waypoints, I converted waypoints to vehicle's local coordinates (instead of global coordinates). Therefore, I carried out all computation is terms of the local coordinates, e.g. I evaluated the cross track error applying the polynomial at x=0, in terms of distance from the origin.


### Model Predictive Control with Latency 

> _The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency._  

In this project, I apply a 200 ms elapsed duration. Being the latency shifting the reference point 100 ms ahead, I also shifted forward my starting point averaging the first two values of actuators variables to center between them, i.e. 100 ms after a hypothetical no-latency expected starting point.  



