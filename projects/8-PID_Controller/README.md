# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

This is a C++ implementation of a simple PID controller based on cross-track error (CTE). Controller is used to control steering of a car, but general principle can be used for any control correction (e.g. speed).

## P, I, D components of the PID Controller

- `P` component is proportional to cross-track error and it determines how fast the car control system responds to the cross-track error. It is responsible for getting the vehicle close to the target path but it will not reach the exact target due to overshooting and oscillations.

- `D` component is temporal difference of the CTE. Using this derivative term along with the P component, the vehicle can avoid overshooting of target trajectory and get more stable behavior.

- `I` component is proportional to the sum of all the cross-track errors the vehicle ever observed. It helps compensate for biases (e.g. caused by steering drift) which prevent the P-D controller from reaching the center line.

## Hyperparameter tuning

The final hyperparameters (`P`, `I`, `D` coefficients) were chosen manually by trial and error. Other possible approach would be to use *coordinate ascent* algorithm.

I used following logic:

1. Start with setting `D` and `I` to zero and only tune the `P` term until the vehicle has stable oscillation around center of the lane which is the reference position.

2. Tune the `D` term until oscillations of the vehicle are smoothened out (i.e. overshooting is void).

3. Tune the `I` term until reaching minimal number of oscillations. Since `I` component considers sum of all cross-track errors from the starting point of the car, we normally use very small value.


I selected following values:
  - `P = 0.2`
  - `I = 0.0002`
  - `D = 3.0`

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
5. Run the simulator
