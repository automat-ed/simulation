#include "ros/ros.h"
#include "sensors/inertialUnit.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/Motor.hpp"
#include "webots/Robot.hpp"

int main(int argc, char **argv) {
  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters
  std::string imu_name;
  nh.getParam("imu", imu_name);
  int step_size;
  nh.getParam("step_size", step_size);

  // Set up Webots
  webots::Robot *robot = new webots::Robot();

  // Get webot devices
  webots::InertialUnit *wb_imu = robot->getInertialUnit("imu");

  // Instantiate sensor wrappers
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(wb_imu, &nh);

  while (robot->step(step_size) != -1) {
    imu.publishImuQuaternion();
  }

  // Clean up
  delete robot;
  return 0;
}