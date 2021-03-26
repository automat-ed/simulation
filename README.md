# AutomatED Simulation

[![Continuous Integration](https://github.com/automat-ed/simulation/actions/workflows/ci.yaml/badge.svg)](https://github.com/automat-ed/simulation/actions/workflows/ci.yaml)

<p align="center">
<img src="https://user-images.githubusercontent.com/38025909/112568551-8c3b1000-8dda-11eb-963f-cbf22350798d.png" />
</p>

## Introduction
This repository is a [ROS 1 Noetic](https://wiki.ros.org/noetic) package. It contains various components (discussed below) that together form a platform ontop of the [Webots simulator](https://www.cyberbotics.com/#cyberbotics) to allow our [software](https://www.github.com/automat-ed/autonomous) to run in a simulated environment. Such components include sensor abstractions that publish to ROS topics, a robot model, keyboard controller, and numerous testing environments with obstacles, ice and humanoids.

Currently the Robot model has 6 sensors:
1. [Accelerometer](https://github.com/automat-ed/simulation/blob/main/lib/sensors/src/accelerometer.cpp)
2. [GPS](https://github.com/automat-ed/simulation/blob/main/lib/sensors/src/gps.cpp)
3. [Gyroscope](https://github.com/automat-ed/simulation/blob/main/lib/sensors/src/gyro.cpp)
4. [Inertial Unit](https://github.com/automat-ed/simulation/blob/main/lib/sensors/src/inertialUnit.cpp)
5. [Lidar](https://github.com/automat-ed/simulation/blob/main/lib/sensors/src/lidar.cpp)
6. [Wheel Odometry](https://github.com/automat-ed/simulation/blob/main/lib/sensors/src/wheelOdom.cpp)

## Prerequisites
To run this package you will need to have installed:
* [ROS 1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* [Webots (R2021a)](https://www.cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Click the links above to go to their respective installation pages.

## Installation
The following instructions details how to install the package and its dependencies. We assume you have installed the packages listed in the [Prerequisites section](#prerequisites) and that you have sourced your ROS installation (using `source /opt/ros/noetic/setup.bash` or similar).
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

4. Set WEBOTS_HOME environment variable to the webots directory obtained from installing Webots (typically `/usr/local/webots` on Linux):
    ```bash
    echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
    ```
    Or if you are using zsh:
    ```bash
    echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.zshrc
    ```
5. Add to your `.bashrc` file (or `.zshrc` if you use zsh):
    ```bash
    export LD_LIBRARY_PATH="${WEBOTS_HOME}/lib/controller:$PATH"
    ```
6. Build workspace (`catkin_make` will also work if you don't have [`catkin_tools`](https://catkin-tools.readthedocs.io/) installed)
    ```bash
    catkin build
    ```
5. Source workspace
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

## Directory Structure
This section tries to explain the general structure of this repository. For more detailed information, please take a look at the code.
* `.github/workflows`: contains the Github Actions config file for the [Continuous Integration pipeline](https://github.com/automat-ed/simulation/actions/workflows/ci.yaml)
* `launch`: Contains the launch files relevant to launching Webots
* `lib`: Defines three C++ libraries grouped by functionality:
    * `sensors`: This library contains classes that abstract over the details of extracting sensor data from the simulation and publishing them to ROS topics.
    * `steering` This library contains a class that allows us to convert linear and angular velocities into individual motor commands for a differential steered robot.
    * `utils`: Contains miscellaneous classes.

    Most of the classes defined in here will also define additional ROS parameters specific to each class. These must be passed in when launching the external controller as it is all one node.
* `protos`: Contains the [PROTO file](https://www.cyberbotics.com/doc/reference/proto-definition#!) for the robot model
* `src`: Contains the `.cpp` file for the external controller. This is added to the robot model and is the sole entry point into the simulation. It is also a ROS node. All classes in `lib` get passed a ROS Handler to this ROS node.

## Topics

### Publishers
Below we list the set of topics that are published to by the external controller

Topic name | Message Type | Description
---------- | ------------ | -----------
`/accelerometer/data` | `sensor_msgs/Imu` | Linear acceleration with noise
`/acelerometer/ground_truth` | `sensor_msgs/Imu` | Ground truth linear acceleration
`/gps/coordinates` | `sensor_msgs/NavSatFix` | GPS coordinates (WSG84) with noise
`/gps/ground_truth/coordinates` | `sensor_msgs/NavSatFix` | Ground truth GPS coordinates (WSG84)
`/ground_truth/pose` | `nav_msgs/Odometry` | Ground truth position of the robot model
`/gryo/data` | `sensor_msgs/Imu` | Angular velocity with noise
`/gyro/ground_truth` | `sensor_msgs/Imu` | Ground truth angular velocity
`/imu/data` | `sensor_msgs/Imu` | Orientation with noise
`/imu/ground_truth` | `sensor_msgs/Imu` | Ground truth orientation
`/lidar/data` | `sensor_msgs/LaserScan` | Lidar laser scan with noise
`/lidar/ground_truth` | `sensor_msgs/LaserScan` | Ground truth laser scan
`/tf_static` | `tf2_msgs/TFMessage` | Static stransforms of each sensor
`/wheel_odom/data` | `geometry_msgs/TwistWithCovarianceStamped` | Linear velocity with noise
`/wheel_odom/ground_truth` | `geometry_msgs/TwistWithCovarianceStamped` | Ground truth linear velocity

### Subscriptions
Below we list the list of topics t

Topic name | Message Type | Description
---------- | ------------ | -----------
`/cmd_vel` | `geometry_msgs/Twist` | Linear and angular velocities to give to the robot
