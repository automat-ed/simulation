# AutomatED Simulation

[![Continuous Integration](https://github.com/automat-ed/simulation/actions/workflows/ci.yaml/badge.svg)](https://github.com/automat-ed/simulation/actions/workflows/ci.yaml)

<p align="center">
<img src="https://user-images.githubusercontent.com/38025909/112568551-8c3b1000-8dda-11eb-963f-cbf22350798d.png" />
</p>

## Introduction
This repository is a [ROS 1 Noetic](https://wiki.ros.org/noetic) package. It contains various components (discussed below) that together form a platform ontop of the [Webots simulator](https://www.cyberbotics.com/#cyberbotics) to allow our [software](https://www.github.com/automat-ed/autonomous) to run in a simulated environment. Such components include sensor abstractions that publish to ROS topics, a robot model, keyboard controller, and numerous testing environments with obstacles, ice and humanoids.

## Prerequisites
To run this package you will need to have installed:
* [ROS 1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* [Webots (R2021a)](https://www.cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Click the links above to go to their respective installation pages.

## Installation
The following instructions will details how to install the package and its dependencies. We assume you have installed the packages listed in the [Prerequisites section](#prerequisites) and that you have sourced your ROS installation (using `source /opt/ros/noetic/setup.bash` or similar).
1. Create a workspace
    ```bash
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    ```
2. Clone the repository
    ```bash
    git -C src clone git@github.com:automat-ed/simulation.git
    ```
3. Install package dependencies using [`rosdep`](https://wiki.ros.org/rosdep)
    ```bash
    rosdep install --from-paths src --ignore-src -y
    ```
4. Source workspace
    ```bash
    source devel/setup.bash
    ```
    
    Or if you are using zsh:
    ```bash
    source devel/setup.zsh
    
## Running
Running this package involves running two programs at the same time in separate terminals:
1. Webots
2. the external controller

### Running Webots
To run webots, first open a new terminal:
1. Navigate to your workspace
    ```bash
    cd ~/ros_ws
    ```
2. Source the workspace
    ```bash
    source devel/setup.bash
    ```
3. Launch Webots (with the `square.wbt` world)
    ```bash
    roslaunch simulation webots.launch world:=$(eval pwd)/src/simulation/worlds/square.wbt
    ```
    
The [`webots.launch`](https://github.com/automat-ed/simulation/blob/main/launch/webots.launch) launch file accepts three command-line arguments:
1. `world`: an absolute path to the world file to load into Webots
2. `mode`: use to specify the simulation mode to start Webots in; must be one of `{pause, realtime, run, fast}`
3. `no-gui`: determines whether a minimal gui should be launched for Webots or not

### Running the External Controller
When you run webots, you will notice that your specified world will be loaded, but nothing happens. This is because the external controller is not run. 

Open a new terminal:
1. Navigate to your workspace
    ```bash
    cd ~/ros_ws
    ```
2. Source the workspace
    ```bash
    source devel/setup.bash
    ```
3. Launch the external controller
    ```bash
    rosrun simulation full_controller
    ```
    
After running the controller, you should see the lights on the robot model light up (red for the tail lights and white for the head lights).


The `full_controller` node takes two main parameters:
1. `step_size`: determines the [step size of the webots world](https://cyberbotics.com/doc/guide/controller-programming?tab-language=c++#the-step-and-wb_robot_step-functions)
2. `use_keyboard_control`: determines whether to run the keyboard controller, enabling you to control the robot model with the arrow keys (the Webots world must be in focus to work i.e if your arrow keys aren't moving the robot, try clicking on the robot model and try again).
