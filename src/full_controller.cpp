#include "ros/ros.h"
#include "sensors/gps.hpp"
#include "sensors/inertialUnit.hpp"
#include "sensors/lidar.hpp"
#include "sensors/accelerometer.hpp"
#include "webots/Motor.hpp"
#include "webots/Supervisor.hpp"

int main(int argc, char **argv) {
  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters
  int step_size;
  nh.param("step_size", step_size, 32);

  // Set up Webots
  webots::Supervisor *supervisor = new webots::Supervisor();

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(supervisor, &nh);
  AutomatED::GPS gps = AutomatED::GPS(supervisor, &nh);
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(supervisor, &nh);
  AutomatED::Accelerometer accelerometer = AutomatED::Accelerometer(supervisor, &nh);

  while (supervisor->step(step_size) != -1) {
    lidar.publishLaserScan();
    gps.publishGPSCoordinate();
    gps.publishGPSSpeed();
    imu.publishImuQuaternion();
    accelerometer.publishAccelerometerImu();
  }

  // Clean up
  delete supervisor;
  return 0;
}
