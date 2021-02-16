#include "ros/ros.h"
#include "sensors/lidar.hpp"
#include "webots/Lidar.hpp"
#include "sensors/gps.hpp"
#include "webots/GPS.hpp"
#include "sensors/inertialUnit.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/Motor.hpp"
#include "webots/Robot.hpp"

int main(int argc, char **argv) {
  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters
  std::string lidar_name;
  nh.getParam("lidar_name", lidar_name);
  std::string gps_name;
  nh.getParam("gps_name", gps_name);
  std::string imu_name;
  nh.getParam("imu_name", imu_name);
  int step_size;
  nh.getParam("step_size", step_size);

  // Set up Webots
  webots::Robot *robot = new webots::Robot();

  // Get webot devices
  webots::Lidar *wb_lidar = robot->getLidar("lidar");
  webots::GPS *wb_gps = robot->getGPS("gps");
  webots::InertialUnit *wb_imu = robot->getInertialUnit("imu");

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(wb_lidar, &nh);
  AutomatED::GPS gps = AutomatED::GPS(wb_gps, &nh);
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(wb_imu, &nh);

  while (robot->step(step_size) != -1) {
    lidar.publishPointCloud();
    gps.publishGPSCoordinate();
    gps.publishGPSSpeed();
  }

  // Clean up
  delete robot;
  return 0;
}