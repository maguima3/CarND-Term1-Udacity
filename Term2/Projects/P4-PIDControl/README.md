# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


## Project Description

In this project, I have implemented a PID controller in C++ to maneuver the vehicle of the simulator around the race track.
The simulator provides the cross-track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

More precisely, I have used a PID controller to adjust the steering angle of the vehicle around the track, and a PD controller to adjust
the throttle.

## PID Controller

A proportional-integrative-derivative controller (PID controller) is a control loop feedback¡k mechanism widely used in industrial control 
systems and a variety of other applications requiring continuously modulated control (such as the steering angle of the vehicle in the simulator!)

A PID controller continuously calculates an error value e(t) as the difference between a desired set point (SP) 
and a measured process variable (PV) and applies a correction based on proportional, integral and derivative terms (which give the controller its name).
In this particular project, the error value (CTE) is provided directly to the controller.
The block diagram shows the principles of how the three terms are generated and applied.

![alt text](https://radhesh.files.wordpress.com/2008/05/pid.jpg "PID controller block diagram")

* The proportional term considers how far PV is from SP at any instant in time.
Its contribution to the controller output signal is based on the size of e(t) only at time t.
As e(t) grows or shrinks, the influence of the proportional term grows or shrinks immediately and proportionately.
Setting the proportional gain (Kp) too high causes a controller to repeatedly overshoot the SP, leading to oscillation.

* The integral term addresses how long and how far PV has been away from SP.
The integral term is continually summing e(t). Thus, even a small error, if it persists,
will have a sum total that grows over time and the influence of the integral term will similarly grow.
Thus, even when error is so small that the proportional factor is no longer effective, the integral is
collecting error until it is large enough to matter. In fact, part of the integral's function is to eliminate steady-state offset.
The downside to the integral factor is that it strongly contributes to controller output overshoot past the target setpoint.
The shorter the integral time, the more aggressively the integral works.

* The derivative term considers how fast, or the rate at which, error (or PV) is changing at the current moment
It is a best estimate of the future trend of the error, based on its current rate of change.
The effect of the derivative is to counteract the overshoot caused by P and I.
When the error is large, the P and the I will push the controller output.
This controller response makes error change quickly, which in turn causes the derivative to more aggressively counteract the P and the I.
A properly used derivative allows for more aggressive proportional and integral factors.
Larger derivative time makes the derivative more aggressively dampen P and I.


### PID Loop Tuning

Tuning a control loop is the adjustment of its control parameters
(gain/proportional band, integral gain/reset, derivative gain/rate) to optimum values for a target response.
Tuning is part of loop design, usually required if the system oscillates too much, responds too slowly, has steady-state error, or is unstable.

Several methods are available for tuning a PID loop; the choice of method largely depends on whether or not the loop can be taken offline for tuning,
and the system response speed. If the system can be taken offline, the best tuning method often involves subjecting the system to a step change in input,
measuring output as a function of time, and using this response to determine control parameters.

If the system must remain online, one tuning method is to first set I and D values to zero and increase P until loop output oscillates
— then increase I until oscillation stops, and increase D until the loop is acceptably quick in reaching its reference.
A fast PID loop tuning usually overshoots slightly to reach the setpoint more quickly.

Another is known as the Ziegler-Nichols method. This technique also involves setting I and D gains to zero and then increasing P gain
until the loop output starts to oscillate. Document critical gain Kc and the oscillation period of the output Pc before adjusting
P to 0.5. Kc, I to 0.45 Kc, and D to 0.6. Kc. This proven online method is adequate for loops where quarter-wave decay is acceptable.

## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

* The proportional component is mainly responsible of keeping the vehicle on the track, steering the car towards the center of the lane. A high proportional gain (Kp) makes the car oscillate. A small gain makes the car unable to stay on the track lanes on sharp curves.

The video bellow shows the effects of a P-Controller with a high proportional gain (Kp = 0.8)

[![PIDController](http://img.youtube.com/vi/kF9jtjJrc60/0.jpg)](https://youtu.be/kF9jtjJrc60 "PID Controller - Click to Watch!")


The video bellow shows the effects of a P-Controller with a low proportional gain (Kp = 0.01)

[![PIDController](http://img.youtube.com/vi/dDICXo1kRqU/0.jpg)](https://youtu.be/dDICXo1kRqU "PID Controller - Click to Watch!")


* The integral component is mainly in charge of keeping the vehicle in the middle of the lane, helping the car being more stable.
A small integral time (N_i) makes the car react faster the accumulative cross track error. As the value of the integral term can be pretty big, 
the Ki gain is usually small.

* The derivative component helps the car turning on sharp curves. High derivative gain (Kd) and time (N_d) values produce faster reaction to track changes.


The effects of the proportional, integral and derivative components I have observed are consistent with the theoretical description of the terms.



### Describe how the final hyperparameters were chosen.

I have manually tuned the hyperparametes as described above.

The final parameters of the steering PID controller are:

* Kp = 0.07
* Ki = 0.003
* Kd = 0.8
* N_i = 5
* N_d = 1

The final parameters of the throttle PD controller are:

* Kp = 0.1
* Ki = 0.0
* Kd = 0.4
* N_i = 0
* N_d = 1  

The final configuration allows the vehicle to drive around the track at ~50 Mps rather smoothly.

Here is the video that shows the behavior of the car around the track:
[![PIDController](http://img.youtube.com/vi/8jfYTeX0J9Q/0.jpg)](https://youtu.be/8jfYTeX0J9Q "PID Controller - Click to Watch!")


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

