# Model Predictive Control (MPC)
*Self-Driving Car Engineer Nanodegree Program*

Our aim is to build a controller to drive a car in a simulation environment by following a predefined path on a global map. The actuation commands will be provided by a Model Predictive Controller (MPC). 

The most interesting feature of an MPC is that can "see" in the near future within a time horizon, which means that the actuator commands of steering and throttle can be planned in advance according to the desired pathway. At each action of the MPC, the controller reads in the desired position of the car within this timeframe and finds the optimal set of future actuations so that the predicted trajectory (modelled by a kinematic model, in this case) best matches the pathway. 

---

## Implementation
#### The Model
The model is based on the work presented in [*Kong et al., 2015*](http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf).
We use the bycicle kinematic model to describe the motion of the car: "bycicle" because the car is modelled as if there were only one front and one back wheel placed half-width, "kinematic" because the model only takes into account kinematic quantities and is unaware of forces and inertia. 

The state of a car is read from the simulator, and it is defined as `(x, y, psi, v)`, that is, x and y coordinates, heading and speed in the fixed frame of reference of a global map.

A state `(x, y, psi, v)` evolves into a state `(x1, y1, psi1, v1)` after an infinitesimal time `dt` according to the update equations

	x1 = x + v * cos(psi) * dt
	y1 = x + v * cos(psi) * dt	  
	psi1 = psi + v / Lf * delta * dt
	v1 = v + a * dt
`Lf` is the distance between the front wheel(s) and the center of mass of the vehicle, `a` is the acceleration measured at the initial state, and `delta` is a parameter to take into account tyre slip during steering.

#### Timestep Length and Elapsed Duration
This time horizon `T` is evenly sliced up into `N` timesteps of length `dt`. I set `dt = 100 ms` and `N = 12` upon trial and error: a time horizon of around `1 s` is appropriate to see a curve ahead with sufficient warning for a constant driving speed of around `30 m/s = 65 mph = 110 kph`.

Setting these two parameters goes hand in hand with the definition of the cost function described later.

#### Polynomial Fitting and MPC Preprocessing
The MPC operates in the local coordinate system of the car (x axis is heading of car). The global waypoints read from the simulator are shifted and then rotated to be aligned with the car centre of mass and heading. Then, we fit a 3rd order polynomial to the waypoints to obtain a continuous desired trajectory in the local reference frame.

#### Model Predictive Control with Latency
The result of the fit is a desired trajectory, and we must now find the best sequence of actions to take to follow it. The action to perform *now* is going to be the first of this sequence of actions… but in the presence of latency we'd better predict what is going to be the state of the car the moment the action will be performed. To take this into account, we can simply redefine the *now* by evolving the car state by a time equal to the expected latency according to the bycicle kinematic model as coded in `main.cpp 132–140`. 

The predicted future state is then sent together with the desired trajectory (actually, its fit coefficients) to MPC, which finds the optimum signals to send to throttle pedal and steering wheel. The optimisation is performed by constrained minimisation of the cost function defined in `MPC.cpp 40–56`, which is the sum of different contributions (in order):

- cross-track error
- heading error
- speed error
- a regularisation term that minimises the steering and throttle
- two smoothing terms that prevent sudden jumps in steering angles and throttle from one timestep to the next
 
To obtain a meaningful control of the car, these contributions to the total cost must be appropriately weighted by multiplication parameters, which have been chosen by a mix of physical considerations (e.g., adjust a jerky driving behaviour by increasing the steering angle smoothing coefficient) and trial and error.

## Simulation
The vehicle can drive a lap around the track at a constant speed of `30 m/s`. It shows some proneness to oscillate around the desired trajectory, which translates into crashing on the curb at higher speeds. 70mph is not too bad though, and certainly on par if not better than any other controller used until now in this term :) 

To improve the current model I can think of making the desired speed a function of the steering angle (even better, the predicted steering angle at some point in the future), so that the car can go full throttle on a straight line and approach curves at safe speeds

---
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
