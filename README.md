# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Provided are the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Details of Path Planner

### Prediction main.cpp lines 319-379
The planner takes the provided fusion data. It analyses for each car on which lane it drives, its velocity and longitudinal distance from the ego vehicle in the prediction section. The prediction does not predict behavior of the traffic in "long term horizon" since the schedule time of the prediction, behavior planning and trajectory generation are the same.(20ms) The prediction is in this case more an observation of a "locally static world".

Output: 
   If a vehicle is detected within defined longitudinal boundaries on ego lane, the closest vehicle and its speed are identified.
   If a vehcile is detected within defined longitudinal boundaries on the lane left or right from the ego lane, bool variables are set, in order to mark the respective lane as occupied.
   
   
### Behavior Planning main.cpp lines 381-456    
   
Based on information provided by prediction a cost function was developed in order to find the optimal ego lane.

The cost function has following components:
1. It prefers to stay in the present lane, by adding 0.01 to the cost of all other lanes.
2. It adds a weighted s-distance of each detected vehicle to the respective lane. 
3. It adds  a weighted velocity of each detected vehicle to the respective lane.

The lane with minimal cost function is identified as the best lane.

Two main behavior branches can be identified:

1. A vehicle detected in front of the ego vehicle 
In this case the planner computes the corresponding acceleration to avoid collision. If the neighbouring lane is free, the lane change would not mean getting further away from the best lane and the minimum time from the last lane change is elapsed, the lane change will be performed.


2. No vehicles detected in front of the ego vehicle
Maximal allowed acceleration is set in order to reach the maximal allowed speed. The ego vehicle performes lane change(s) in order to reach the best lane. 

### Path generation main.cpp lines 456 - 524

Based on map data and Frenet coordinates of next 3 waypoints in s distance of 50,60 and 90 meters, XY coordinates of three waypoints are saved in a waypoints vector. Followingly the waypoints are converted into local vehicle coordinates. Converted waypoints are passed to the spline creator. If a path was passed to the planner in previous cycle, it is going to be reused in order the keep the path smooth(line 310-316). New path points are created (line 496-512) based on x,y position in the spline and the velocity of the ego vehicle. Added points are followingly converted back to global(map) coordinate system and sent to the planner.  

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).



