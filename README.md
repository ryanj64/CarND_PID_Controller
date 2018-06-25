# CarND-Project 2-PID Controller
Self-Driving Car Engineer Nanodegree Program

The goal of this project is to use a PID (Proportional-Integral-Derivative) controller to successfully drive the car around the track without leaving the road, running into obstacles, driving on the curbs, and etc.  The three PID parameters are `Kp (proportional gain)`, `Ki (integral gain)`, and `Kd (derivative gain)`.

The formula to calculate the steering angle using a PID controller is as follows:

`steering_angle = (-(Kp_*p_error)-(Kd_*d_error)-(Ki_*i_error))`

 `p_error`, `d_error`, and `i_error` are calculated using the Cross Track Error (CTE).
 The formulas to calculate the three errors is as follows:

```C
//Equal to the cross track error
p_error = CTE
// Equal to the difference between the current and previous CTE.
d_error = (CTE - Previous_CTE);
// Equal to the sum of all previous CTE values.
i_error = (i_error + CTE)
```
In the next section, I will discuss how I tuned Kp, Ki, and Kd.

## Tuning the PID controller using Twiddle

In the lessons, an algorithm called Twiddle or Coordinate Ascent was used to tune the PID gain parameters.

The initial configuration for the algorithm is as follows:

|   | Kp  | Ki | Kd  |
|---|---|---|---|
| Initial gain values  | 0.00  | 0.00  | 0.00  |
| Initial gain step sizes  | 1.00  | 1.00  | 1.00  |

The algorithm was written using C++.  However, to keep the explanation simple, python code was written below to show the process of Twiddle as explained in the lessons.

```python
# [Kp, Ki, Kd]
parameters = [0.00, 0.00, 0.00]
# [Kp delta, Ki delta, Kd delta]
deltas = [1.00, 1.00, 1.00]

tolerance = 0.1

best_error = run_car_simulator(parameters)

while (sum(deltas) < tolerance)
  for index in range(len(parameters)):
    parameter[index] = parameter[index] + deltas[index]
    error = run_car_simulator(parameters)
    if (error < best_error):
      error = best_error
      deltas[index] = deltas[index] * 1.1
    else:
      parameter[index] = parameter[index] - (2 * deltas[index])
      error = run_car_simulator(parameters)
      if (error < best_error):
        error = best_error
        deltas[index] = deltas[index] * 1.1
      else:
        parameter[index] = parameter[index] + deltas[index]
        deltas[index] = deltas[index] * 0.9
```

However, using the configurations provide in the lessons caused the tuning to get stuck at a local minima that was far from meeting the requirements, so the next step is to try manual tuning.

## Manual Tuning

As a starting point, Ki and Kd parameters are set to zero and only the Kp parameter was changed.  The goal is to get as far as possible with only a proportional controller. The car at best was able to cross the bridge using a Kp of 0.04.  Next the Derivative gain (Kd) will be added and manually tuned.  After tuning the parameter Kd, the car was able to make it all the way around the track. However, more tuning is needed. The chosen value for Kd was 4.50 and produced an error of 2.87656. The derivative term (Kd) helped control the rate of change and thus reduced the cars oscillations.

In the next section, Twiddle will be used in addition to manual tuning.  Where Twiddle will help fine tune the parameters Kp, Ki, and Kd.

## Twiddle and Manual Tuning

In the Twiddle section, the starting gains were all zero and the gain step sizes were all set to 1. Now, the gains and gain step sizes will be initialized based on what was learned in the manual tuning section.

The setup of the initial gain and gain step sizes are as follows:

|   | Kp Gain  | Ki Gain | Kd Gain  |
|---|---|---|---|
| Initial gain values  | 0.04  | 0.00  | 4.50  |
| Initial gain step sizes  | 0.01  | 0.00  | 0.00  |

Kp will be adjusted based on the chosen Kd value to see if adjusting only Kp will produce better results.

After running Twiddle, Kp changed from 0.04 to 0.4624 (Ki and Kd were held constant).  The lowest error was 0.108973 and is a significant improvement over the previously noted error.  The next step is to hold Kp and Kd constant and only change Ki to see if there are improvements.  The gain step size chosen for Ki was 0.0001 and was chosen based on what I learned in the manual tuning process.  

The initial configuration for Twiddle is as follows:

|   | Kp Gain  | Ki Gain | Kd Gain  |
|---|---|---|---|
| Initial gain values  | 0.4624  | 0.00  | 4.50  |
| Initial gain step sizes  | 0.00  | 0.0001  | 0.00  |

Ki changed from 0.00 to 0.000309193 with the lowest error being 0.0835435, so there was a slight improvement.  

The final values chosen for the PID controller gains are shown in the table below.

| Gain  | Value  |
|---|---|
| Kp  | 0.4624  |
| Ki  | 0.000309193  |
| Kd  | 4.50  |

## Conclusion

When I started this project, I realized that speed also played a role in tuning the steering angle.  The PID controller for steering was tuned for a car with max speed of 30 MPH.  The current PID will not work well if the car speeds up to 100 MPH.  The CTE value will have a bigger rate of change at 100 MPH vs. 30 MPH.  Also there are tight curves on the road that the car has to slow down for in order to stay on the road.  If the car is going 100 MPH around a tight curve the car will simply drive off the road or flip over.  A human driver will see a tight curve in advance and slow down. If the speed is still too fast you can feel a force pushing your body perpendicular to the curve.

To help control the speed I created another PID controller to stabilize the speed of the vehicle to 30 MPH.  While some of the curves reduce the speed by a few MPH, it stays relatively consistent throughout the run.

The last point is while Twiddle helped to find the lowest error for the chosen PID parameters, it didn't make for the smoothest ride. Some of the parameters chosen in the manual tuning process had bigger errors, but the car had less aggressive turns.  However, the car at times was very close to going off the road at times.

A recording of the run was taken.  The filename is `run.mp4` and is uploaded to this repository to visualize the outcome.


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
