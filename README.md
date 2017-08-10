# CarND Controls Model Predictive Control Project

The present repository is my solution to the [Udacity Controls MPC Project](https://github.com/udacity/CarND-MPC-Project).

---

## Model

The model is based on following equations, that consider steering angle and acceleration of the vehicle:

~~~~
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
~~~~

The state of the vehicle is composed of its position (x, y), its orientation (psi) and its speed (v).
From those data, we calculate cte and epsi, which represent errors respectively due to distance and orientation versus a reference trajectory.

The reference trajectory is given by waypoints, that we interpolate with a polynomial. In our experiements, a 3rd order was giving reasonable results, even in the curves.
We also ensure all coordinates are transferred from the global referential to the car referential which can easily be done by applying a translation followed by a rotation.
This transformation is required both for our approximation of the trajectory with a polynomial and for calculating the errors based on the y coordinate in the car referential. It is to be noted that those 2 conditions are not necessary for a functional model:
* the trajectory could be defined by a generic (x(i), y(i)) function with each coordinate being a polynom of a continuous variable,
* the error could be calculated as being the closest distance to the trajectory defined above.

The actuators are the throttle and the steering angle. In our model, we convert them to acceleration and actual steering of the car by multiplying by a factor experimentally evaluated.
In reality, those parameters are not be directly proportional but we obtain a satisfactory result based on this simple model. To better define them, we would need to run some experiments and include other parameters in our model, such as gravity, friction forces with road, aerodynamic slowing forces applied to the car…

The model simulates N time steps at dt interval, with a 100ms latency.
For that purpose, the initial values of the actuators are set to their current value, and we choose dt = 100ms, which seems reasonable. It is equivalent to performing an action every 100ms, with 100ms latency, in accordance with the model. A possible optimization would be to calculate the cost at additional intervals within this latency time in which the MPC cannot send new commands.

We choose N = 10, simulating 1 second, which is sufficient to anticipate turns. The main issue in considering more steps is that we can consider a turn over 90°, which cannot be interpolated correctly with a polynom and cannot be represented by a function (since multiple y values can be associated with a single x values when drawn on a graph).
This value depends on the reference speed chosen and the proximity to a turn, the faster the car goes and the closer we are to a turn, the less total time we will be able to simulate correctly with this model.

At each step, the model will optimize the actuators (throttle and steering angle) in order to minimize the total cost of the car trajectory. This cost is composed of the following values:
* the y distance in the car referential to the reference trajectory,
* the orientation difference between the car and the trajectory,
* the difference between the speed of the vehicle and the target speed.
* the difference of values imposed on actuators at consecutive time steps to ensure the actuators are controlled smoothly and that a large control will happen only in case of emergency.

We wanted to ensure that each contribution to the error has an effect to the optimization model so we scaled them to have similar order of magnitudes while still controlling which effect had the most contribution to the error. Indeed, it is more important to be close to the target trajectory than to be at the desired speed.

We were able to run the model at a very high target speed, only decreasing it when getting closer to turns. We identify turns based on the polynomial approximation for the waypoints. By experimenting with the values at x = 50, we can detect those and decrease linearly the target speed to a pre-defined minimal value.


---

# Original Readme

Here below is the original readme of the project.

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
