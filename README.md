# Extended Kalman Filter Project Completed Code
For Self-Driving Car Engineer Nanodegree Program - This is a C++ implementation of the Extended Kalman Filter.

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

## Basic Build Instructions

1. Clone or download this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

Having used Code::Blocks as IDE for this particular project, the following editor settings are recommended:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

I have tried my best to stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html). In case of any discrepancies found, please let me know and I will edit them right away!

## Generating Additional Data

For the ones who have a Matlab license, please feel free to generate your own radar and lidar data- see the [utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Rubrics

### Compiling

##### The Code Should be simple to compile.

Given that the file `CMakeLists.txt` contains all the information for cmake build, it shouldn't take much effort in compiling the files. I had a windows OS which did not respond to my make software installation. Therefore I used Code::Blocks to execute my C++ programs. The output file is saved within the `CarND_Term2_Project1_EKF` folder as `output.txt`.

### Accuracy

##### The px, py, vx, vy output coordinates have an RMSE <= [0.08, 0.08, 0.60, 0.60] when using the file: "sample-laser-radar-measurement-data-1.txt".

Upon running the code, the root mean square error (RMSE) of the Kalman estimate values with respect to the ground truth values were found to be as `[0.0651663, 0.0605391, 0.530409, 0.544205]`.

##### The px, py, vx, vy output coordinates have an RMSE <= [0.20, 0.20, .50, .85] when using the file: "sample-laser-radar-measurement-data-2.txt".

Upon running the code, the root mean square error (RMSE) of the Kalman estimate values with respect to the ground truth values were found to be as `[0.0651663, 0.0605391, 0.530409, 0.544205]`.

### Follows the Correct Algorithm

##### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The sensor fusion algorithm collates the data from LIDAR and RADAR sensors and processes them as per the time stamp of their inputs. The starter code provided to me collected and gathered the given sensor data and my code tried to process this information as per the time stamp in a sequential manner.

##### Your Kalman Filter algorithm handles the first measurements appropriately.

The starting part of the algorithm initializes the state variables as the first point from the measurement data, either from LIDAR or RADAR. This can be changed to something random too, but the Kalman filter algorithm will probably take a longer time to approach to the true value of the state.

##### Your Kalman Filter algorithm first predicts then updates.

The Kalman filter algorithm is designed in such a way that it first goes through the prediction step, which is coded as a part of a `Predict()` function in the `KalmanFilter` class. The algorithm follows from another class called `FusionEKF` which calls a `KalmanFilter` class and initializes its variables to zero. After the `Predict()` function, it either calls for an `Update()` function or an `UpdateEKF()` function for the measurement update.

##### Your Kalman Filter can handle radar and lidar measurements.

The updating functions within `FusionEKF` class are coded in such a way that differentiates between LIDAR and RADAR measurement data. The computations for both are different, since LIDAR gives an update in Cartesian coordinates and RADAR gives an update in Polar coordinates.   

### Code Efficiency

##### Your algorithm should avoid unnecessary calculations.

The C++ codes are optimized in such a way that no two computations are repeated. In case one finds an area of opportunity for optimization, please let me know and I would like to make the necessary changes.
