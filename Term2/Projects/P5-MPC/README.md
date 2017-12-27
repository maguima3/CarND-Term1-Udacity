# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---

# Project Specifications

## Submission video

[![MPC](http://img.youtube.com/vi/A6y9MnUT5pI/0.jpg)](https://youtu.be/A6y9MnUT5pI "MPC - Click to Watch!")

## MPC Intro

Model predictive control (MPC) is an advanced method of process control.
Controllers rely on dynamic models of the process, most often linear empirical models obtained by system identification.
The main advantage of MPC is the fact that it allows the current timeslot to be optimized, while keeping future timeslots into account.
This is achieved by optimizing a finite time-horizon, but only implementing the current timeslot and then optimizing again, repeatedly.
Also MPC has the ability to anticipate future events and can take control actions accordingly.

Note: PID controllers do not have this predictive ability.

To sum up, MPC is a multivariate control algorithm that uses:

* an internal dynamic model of the process,
* a history of past control moves and
* an optimization function J over the receding prediction horizon
to calculate the optimum control moves.

MPC has been successful in semi-autonomous and autonomous driving. In fact, MPC is suited for this class of problems since it can handle
multi-input multi-output systems with input and state constraints while taking into account the nonlinear dynamics of the vehicle.


## The Model

In this project I have used a simple kinematic bicycle model, which is less computationally expensive than existing methods and performs fairly
well in the simulator.

The nonlinear continuous time equations that describe the kinematic bicycle model in a moving frame (assuming that the angle of the current velocity
of the center of the mass with respect to the logitudinal axis of the car is zero) are:

* (1) `x = x0 + v0 * cos(psi) * dt`
* (2) `y = y0 + v0 * sin(psi) * dt`
* (3) `psi = psi0 + v/Lf * delta * dt`
* (4) `v = v0 + a * dt`
* (5) `cte = cte0 + (f(x0) - y0) * v0  * sin(epsi) * dt`
* (6) `epsi = epsdi * ((psi0-psio0) + v0 / Lf * delta* dt`

where x and y are the coordinates is the centers of mass, psi is the heading angle, v is the speed of the vehicle,
Lf represents the distance from the center of the mass of the vehicle to the front axle, cte determines the cross-track error,
and epsi the orientation error.

The control inputs are the front steering `delta` (we assume rear steering = 0) and the acceleration `a`.

Thus, a six state model input vector is used:
[x, y, psi, v, cte, psie]

The function used to optimize the control values can be found in the lines 50-70 of MPC.cpp.

```java

    // Reference State Cost
    fg[0] = 0;
    
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.
    for (int t = 0; t < N; ++t) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[epsi_start + t], 2); //10
      fg[0] += CppAD::pow(vars[v_start + t] - v_ref, 2);
    }
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; ++t) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2); //100
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }
    // Minimize the value gap between sequential actuations. (Smooth)
    for (int t = 0; t < N - 2; ++t) {
      fg[0] += 1000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); //700
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

Forcing the controller to minimize the gap between sequential steering actuations has been decisive in driving the vehicle smoothly around the track.
Finding the correct weights has been a hard point, and I tried a lot to get an acceptable vehicle behavior.
At the beginning I associated the erratic behavior of the car to the latency,
but then I figured out that the incorrect weights of the cost function were causing it (more precisely, the weight I set to ``fg[0] += CppAD::pow(vars[cte_start + i], 2);``  was too big).

## Timestep Length and Elapsed Duration (N & dt)

The final values chosen for N and dt are 10 and 0.2 (10 and 0.1 performs good too, but the car swings a little bit more around the center of the lane).

I have tried bigger and smaller values of N and dt, which led to erratic behavior.

## Polynomial Fitting and MPC Preprocessing

Before calling the MPC method I do two things (main.cpp lines 110 - 130):

* Predict the current state 100 ms into the future (see more bellow)
* Transform the way points into vehicle's coordinate system.
This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now
at the origin (0, 0) and the orientation angle is also zero.

```java
          // Transform waypoints into car's coordinate system
          for (int i = 0; i < n_waypoints; ++i) {
            double xn = ptsx[i] - px;
            double yn = ptsy[i] - py;
            x_waypoints[i] = xn * cos(psi) + yn * sin(psi);
            y_waypoints[i] = yn * cos(psi) - xn * sin(psi);
          }
```

* Fit the waypoints into a third order polynomial, since they fit most of roads

## Model Predictive Control with Latency

Introducing 100 ms 'delay' in the car's state before predicting the trajectory results in a better and more stable behavior of the car.
I introduce the latency time before transforming the waypoints from global to vehicle's coordinates.

Note: I have used dt > latency time. 

```java
          // Predict current state 100 ms into the future to account for latency
          // Assuming constant the previous actuator commands
          double latency = 0.1;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi += v * delta/2.67 * latency;
          v += acceleration * latency;
```


---
# Original README from Udacity
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
