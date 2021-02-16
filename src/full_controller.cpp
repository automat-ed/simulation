#include "ros/ros.h"
#include "sensors/lidar.hpp"
#include "webots/Lidar.hpp"
#include "webots/Motor.hpp"
#include "webots/Robot.hpp"

int main(int argc, char **argv) {
  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters
  std::string lidar_name;
  nh.getParam("Velodyne_VLP16", lidar_name);
  int step_size;
  nh.getParam("step_size", step_size);

  // Set up Webots
  webots::Robot *robot = new webots::Robot();

  // Get webot devices
  webots::Lidar *wb_lidar = robot->getLidar("Velodyne_VLP16");

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(wb_lidar, &nh);

  while (robot->step(step_size) != -1) {
    lidar.publishPointCloud();
  }

  // Clean up
  delete robot;
  return 0;
}