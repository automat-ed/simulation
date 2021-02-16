#include "ros/ros.h"
#include "sensors/gps.hpp"
#include "webots/GPS.hpp"
#include "webots/Motor.hpp"
#include "webots/Robot.hpp"

int main(int argc, char **argv) {
  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters
  std::string gps_name;
  nh.getParam("gps_name", gps_name);
  int step_size;
  nh.getParam("step_size", step_size);

  // Set up Webots
  webots::Robot *robot = new webots::Robot();

  // Get webot devices
  webots::GPS *wb_gps = robot->getGPS("gps");

  // Instantiate sensor wrappers
  AutomatED::GPS gps = AutomatED::GPS(wb_gps, &nh);

  while (robot->step(step_size) != -1) {
    gps.publishGPSCoordinate();
    gps.publishGPSSpeed();
  }

  // Clean up
  delete robot;
  return 0;
}