# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Provided are the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Details of Path Planner

The planner takes the provided fusion data. It analyses for each car on which lane it drives, its velocity and longitudinal distance from the ego vehicle. Based on this information a cost function was developed in order to find the optimal ego lane. Based on longitudinal distance from vehicles in front of the ego vehicle and speed limit, the proper acceleration is set. Using the spline function from section Tips, a set of new points of egos path are created and sent to the simulator.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).



