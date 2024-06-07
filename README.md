![GitHub issues open](https://img.shields.io/github/issues/nocholasrift/mpc_trajectory_tracking)
![GitHub forks](https://img.shields.io/github/forks/nocholasrift/mpc_trajectory_tracking)
![GitHub stars](https://img.shields.io/github/stars/nocholasrift/mpc_trajectory_tracking)

# Nonlinear Model Predictive Controller

## About
This repository contains an MPC Trajectory Tracker based on the [mpc_ros](https://github.com/Geonhee-LEE/mpc_ros) library developed by Geonhee-LEE et. al. The MPC itself is written over the [IPOPT](https://coin-or.github.io/Ipopt/) nonlinear solver and integrated with ROS (Robot Operating System). The model used for trajectory tracking within the MPC is the unicycle model, but there are plans to expand this out to double / triple integrators as well, and even to aerial vehicles. These features will come out when I either need them for a project or when I get the free time to develop them.

## Installation
**NOTE: Tested in Ubuntu 16.04/18.04/20.04 in ROS Kinetic/Melodic/Noetic.**

1. Install ROS

2. [Install IPOPT](https://coin-or.github.io/Ipopt/INSTALL.html). So far only tested on IPOPT version 3.14.4, no guarantees that it works on any other version.
    - The steps I use to install IPOPT can be found in the [install_ipopt.sh](install_ipopt.sh) script.

4. Clone repository into a `catkin_ws` and compile with either `catkin build` or `catkin_make`
    - Note: The following line was added to the `CMakeLists.txt` in case the user doesn't want to add `/usr/local/lib` to `LD_LIBRARY_PATH`:
    ```
    link_directories(/usr/local/lib)
    ```
    If you're IPOPT install is located elsewhere, I'd recommend changing that directory to wherever your IPOPT libraries are if you are having problems linking.

## Running

You can run the MPC by launching the following launch file: 
```
roslaunch mpc_trajectory_tracking jackal_mpc_track.launch
```

All the MPC needs is a reference trajectory, then it will publish a Twist message to the `cmd_vel` topic.

>   *Note: You can change the default odometry topic via the `odomTopic` argument in the launch file.*

You can send a trajectory to one of two topics:
    
1. `/reference_trajectory`: Will reset the trajectory timer and track the trajectory starting at time=0s.

2. `/reference_trajectory_no_reset`: Same as `reference_trajectory` but will not reset the trajectory timer. This is useful for appending to a trajectory currently being tracked.

## Maintenance

I am actively adding features / fixing bugs in this repository. With that said, if you would like to use this repository and are having any troubles, please feel free to submit an issue and I will most likely see it within a few days.
