# Particle Filter

This code implements a Particle Filter to find a kidnapped vehicle on a map. 

---

## Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This project implements a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step, the filter also gets observation and control data. 

## Particle Filter Implementation
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The meat of the particle filter is in `particle_filter.cpp` in the `src` directory. The file contains the `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code does.

The file `src/main.cpp` contains the code that interfaces to the simulator and will actually be running the particle filter and calling the associated methods.

## Dependencies
* [Udacity Self-Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
    * Linux: run the [`install-ubuntu.sh`](install-ubuntu.sh) script in the repository
    * Mac: run the [`install-mac.sh`](install-mac.sh) script in the repository
    * Windows: use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)
* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: install [Xcode command line tools](https://developer.apple.com/xcode/features/) to get make (type `xcode-select --install` in Terminal)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - install [Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build and Run Instructions

1. Clone this repo
2. Install uWebSocketIO as indicated [above](#dependencies)
3. `./clean.sh`
4. `./build.sh`
5. `./run.sh`
6. Start Udacity simulator application
    1. Select resolution, graphics quality, etc., and press "Play!" 
    2. Select "Project 3: Kidnapped Vehicle"
    3. Press "Start"

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


## uWebSocketIO Protocol Usage

If you wish to modify the communication between the simulator and filter, here is the main protocol that main.cpp uses for uWebSocketIO.

**INPUT**: values provided by the simulator to the C++ program

Sense noisy position data from the simulator

```
["sense_x"] 
["sense_y"] 
["sense_theta"] 
```

Get the previous velocity and yaw rate to predict the particle's transitioned state
```
["previous_velocity"]
["previous_yawrate"]
```

Receive noisy observation data from the simulator, in a respective list of x/y values
```
["sense_observations_x"] 
["sense_observations_y"] 
```

**OUTPUT**: values provided by the c++ program to the simulator

Best particle values used for calculating the error evaluation
```
["best_particle_x"]
["best_particle_y"]
["best_particle_theta"]
``` 

Optional message data used for debugging particle's sensing and associations for respective (x,y) sensed positions 
```
["best_particle_associations"]
["best_particle_sense_x"] <= list of sensed x positions
["best_particle_sense_y"] <= list of sensed y positions
```