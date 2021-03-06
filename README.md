# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
---

[//]: # (Image References)

[image0]: ./pictures/architecture.png "processing flow"
[image1]: ./pictures/trajectory_generation.png "trajectory generation"
[image2]: ./pictures/speed_scheduling.png "speed scheduling"
[image3]: ./pictures/behavior_planning.png "behavior planning"

# Introduction

The target of the project is to generate paths (way-points) for a simulated car to drive as fast as possible without violating the following constraints:
- Maximum speed of 50 mph
- No collision with other cars on the road
- Stay in lane, except the 3 sec. tolerance for lane changing
- Maximum absolute value for acceleration of 10 𝑚/𝑠^2
- Maximum jerk of 10 𝑚/𝑠^3

The prior knowledge the program got are the following
- Map way points (coarse)
- The runway is circular with circumference of 6945.554 m 
- Totally 3 lanes (Fixed)

During the run time, the program received the following signals 
- Sensor fusion data, including other cars’ position and velocity 
- Ego vehicle’s latest position and velocity
- Remained path waypoints that haven’t been used


# Architecture

There are two main block of function in the process
- Behavior planning (`code line #246 ~ #513 in main.cpp`)
- Path planning (`code line #576 ~ #682 in main.cpp`)

The behavior planner receives the sensor fusion data at each time step/iteration, performs the optimization process, and output the target lane (`lane`) and target speed (`set_vel`). After deciding the target lane and speed, the path planner will try to fill up the gap between the last generated path and the target lane (`lane`) using spline and sample the waypoints according to the speed at each step, where the speed at each step will slightly increase/derease toward the desired speed (`set_vel`).

![alt text][image0]

Fig. 1 Processing flow of the path-planning program

The following sections will describe the two block separately. 



## Path Planning 

The path plannning block can be sub-devided into two parts
- Trajectory generation (`code line #576 ~ #635 in main.cpp`)
- Speed scheduling (`code line #644 ~ #682 in main.cpp`)

### Trajectory Generation

The target of the trajectory generation is to generate a smooth trajectory (`spline`) according to the desired lane and given map waypoints. The standard procedure is listed below
1. Decide a reference point to start the new path (usually the last point of the previous path)
1. Chose last 2 points from the previous path and 3 points on desired lane (may be different lane as the one the first 2 points at) that is 30m, 60m, and 90m ahead the reference point (in Frenet-s) as anchor points.
1. Convert the anchor points from Frenet coordiante** to Global Cartesian coordinate (using `getXY()` at `#131~#177` in `helpers.h`, given _map waypoints_)
1. Transform all the anchor points to _local coordinate_ with respect to reference point and reference angle (to simplify the speed scheduling process)
1. Use 5 anchor points, in local coordinate, to generate the spline


However, the map waypoints are quit sparse that it's not sufficient to calculate the correct transformation between Frenet and Cartesian coordinate (i.e. `getXY()` at `#131~#177` in `helpers.h` does not generate precise (x,y) from (s,d) with original map waypoints). The result of this is a trajectory that sometimes biased from the center line of the lane if we use the erronic transformed (x,y) points as anchore points to generate the spline. Nevertheless, this problem can be solved by pre-generating the local fine map waypoints. Therefore, the final version of the trajectory generation process is 

1. Decide a reference point to start the new path (usually the last point of the previous path)
1. Chose last 2 points from the previous path and 3 points on desired lane (may be different lane as the one the first 2 points at) that is 30m, 60m, and 90m ahead the reference point (in Frenet-s) as anchor points.
1. ***Generate **local fine map waypoints** using splin, with original map waypoints as anchor points (`get_local_fine_map()` at at `#234~#310` in `helpers.h`)***
1. Convert the anchor points from **Frenet coordiante** to **Global Cartesian coordinate** (using `getXY()` at `#131~#177` in `helpers.h`, given **local fine map waypoints**)
1. Transform all the anchor points to _local coordinate_ with respect to reference point and reference angle (to simplify the speed scheduling process)
1. Use 5 anchor points, in local coordinate, to generate the spline



Fig. 2 illustrates the process of trajectory generation.



![alt text][image1]

Fig. 2 Trajectory generation using spline


### Speed Scheduling

The final product of the whole path-planning program is a list of points with that the coordinate of each point describes the position of the car at the corresponding time step and the distance between each pair of neighbor points is the speed within the two corresponding steps.

With the parametrized expression of the curve (spline) obtained by the previous procedure, the last task in the program is to sample the mentioned points from the given trajectory. The following step generate the desired list of points with approximated speed profile.



1. Chose a target x (e.g. 30 m forward)
1. Calculate the target y using the trajectory obtained from previous step (the spline, `y=s(x)`)
1. Calculate 𝜃_𝑑𝑒𝑝𝑎𝑟𝑡
1. According to the set_vel and acce_max/acce_min, slightly change the current speed at each iteration
1. At each iteration, project the increment into 𝑥_𝑙𝑜𝑐𝑎𝑙 and generate the y value of the waypoint using spline  





![alt text][image2]

Fig. 3 Speed scheduling and path waypoint generation

## Behavior Planning

The behavior planning desired the optimal action (to chose a lane and speed) at each iteration. For the planner in this project, the action space is restricted to simply chose between three lane (i.e. |a|=3), the desired speed is determined solely according to that if there was close frontal car or not (using `cal_proper_speed()` at `#351~#438` in `helpers.h`). The optimality of the planner is set to find the longest traveling distance at a fixed time horizon, and the feasibility is defineh into other cars.

The planning algorithm implemented here (`code line #246 ~ #513 in main.cpp`) is a **depth-1 uninformed searching algorithm** with early stoping. The searching method is performed by simulating the ego car (`code line #345 ~ #376 in main.cpp`) and other cars (`code line #330 ~ #333 in main.cpp`) forward in time for each chosen lane. The ego car in simulation will run toward the chosen lane and then stay on that lane (`code line #366 ~ #376 in main.cpp`). If there was a close frontal car, the simulation car will use the same strategy as the real decision used (using `cal_proper_speed()` at `#351~#438` in `helpers.h`) to slow down (`code line #345 ~ #363 in main.h`). If a chosen action resulted in a collision before reaching the simulation time horizon, the simulation will stop for that action. 

The final decision of desired lane is of that the lane resulted in longest travel distance one (`code line #406 ~ #429 in main.cpp`). Since the early stopping of the simulation, the action resulted in collision natually produce shorter traveling lenth, the checking of collision is not neccessary.

One minor step left is a filter that limit the ego car to shift only one lane at a time (`code line #483 ~ #491 in main.cpp`), so that the car won't do something acrobatic/non-proper movements.


![alt text][image3]

Fig. 4 Behavior planning through forward simulation




# Simulation Result

## Normal Drive (Satisfy all the constraints required)

Here's the [link to the video (49.5 mph).](https://youtu.be/yQ6OSxGZ7JA)

[![video - 49.5 mph](http://img.youtube.com/vi/yQ6OSxGZ7JA/0.jpg)](https://youtu.be/yQ6OSxGZ7JA)


The simulation result shows the planners behave well that it did not violate any of the constraints.

The ego car spend **5 minutes and 25 seconds** to travel a round (_4.32 miles_), which resulted in an average speed of **47.85 mph**. This shows that the car chose the lane to drive wisely so that it did not need to slow down to avoid crashing into the frontal cars.

In addition, the car also shows some intelligence during the drive.
- Smooth lane changing without slowing down (`0:25 49.5mph video`)
- Change lane early even when the frontal car is way distant from it (`3:33 49.5mph video`)
- Double lane changing, e.g. 02 or 2  0 (`2:16 in 49.5mph video`)




## High-speed Drive (Break the speed limit with top speed of 200 mph)

In order to test the limit of the planners, the following experiment is also conducted to run drive the ego vehicle at the top speed of 200 mph. 

Here's the [link to the video (200 mph).](https://youtu.be/HW9u6UjKj7I)

[![video - 200 mph](http://img.youtube.com/vi/HW9u6UjKj7I/0.jpg)](https://youtu.be/HW9u6UjKj7I)


The result is quit interesting when the car changed to the lane that ultimatly resulting in smoother drive even there is no frontal car right in front of it (`1:54 in 200mph video`).




---   
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

