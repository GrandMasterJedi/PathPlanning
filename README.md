[image1]: ./image/changeLane_5miles.PNG "im1"
[image2]: ./image/frenetCoord.png "im2"

# Demo for Self Driving Path Planning

A vehicle navigates around a virtual highway with traffic created by other vehicles. It tries to drive at the 50 MPH speed limit with the following conditions:
* It drives as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. 
* It avoids hitting other cars at all cost
* It drives inside of the marked road lanes at all times, unless going from one lane to another. 
* It is able to make one complete loop around the 6946m highway in about 5-6 minutes (with average speed of about 80 km/h or 50 MPH it should take about 5 mins and 15 secs ) 
* Its total acceleration is lower than 10 m/s^2 
* Its jerk is lower than 10 m/s^3.

![alt text][image1]

This project is my solution to Term 3, assignment 1 of the Udacity Self-Driving Car Engineer Nanodegree Program. 


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* [spline](http://kluge.in-chemnitz.de/opensource/spline) library
    

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```sh
sudo chmod u+x {simulator_file_name}
```

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
---


## Data

The car's localization and sensor fusion data are given, there is also a sparse map list of waypoints around the highway. 

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)
* ["x"] The car's x position in map coordinates
* ["y"] The car's y position in map coordinates
* ["s"] The car's s position in frenet coordinates
* ["d"] The car's d position in frenet coordinates
* ["yaw"] The car's yaw angle in the map
* ["speed"] The car's speed in MPH

#### Previous path data given to the Planner
//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 

* ["previous_path_x"] The previous list of x points previously given to the simulator
* ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 
* ["end_path_s"] The previous list's last point's frenet s value
* ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
* ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---


## Reflection

The particular path planned I implemented for the vehicle is based on heuristic rules. The model assess the position of my car with respect to all other cars and command my vehicle to accellerate, lower the speed and/or change lane. It uses Frenet coordinates (s,d) to assess distance and lane for all vehicles. Those coordinates that are converted to my car local point of view (Cartesian) for trajectory generation. The Frenet coordinates are as shown in the image below. 
The two main components of the algorithm are behavioral planning and path generation. 
![alt text][image2]

### Behavior Planning 
It assess the position of other vehicles with respect of my vehicle and based on this, the behavioral planned choose to increase/decrease the speed (with controlled jerk/braking rate) and make a lane change. The planner avoid collisions by satisfying the following hard constraints:
1. Accelerate only if no vehicles is within 30 meters
2. Change lane if a vehicle is driving slower in front and no vehicles on the two sides within +- 30 meters of s
3. Deccelerate if the first two conditions are not satisfied


### Path Generation 
This part relates to the calculation of the trajectory based on the speed and lane of my vehicle and also the past path points. It uses the spline.h library to interpolate through the last two points of my path at time t and future points at respectively 30m, 60m and 90m distance. The spline takes local car coordinates (after shift and rotation of Frenet coordinates). Also, the spline is continuous at each time stamp with the the pass trajectory points copied to the new trajectory. 



--- 

## Resource

* Spline library is available [here] (http://kluge.in-chemnitz.de/opensource/spline/)

---

