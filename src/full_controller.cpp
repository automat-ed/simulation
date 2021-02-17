#include "ros/ros.h"
#include "sensors/lidar.hpp"
#include "webots/Lidar.hpp"
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
  nh.param<std::string>("lidar/name", lidar_name, "RobotisLds01");
  std::string imu_name;
  nh.param<std::string>("imu/name", imu_name, "imu");
  int step_size;
  nh.param("step_size", step_size, 32);

  // Set up Webots
  webots::Robot *robot = new webots::Robot();

  // Get webot devices
  webots::Lidar *wb_lidar = robot->getLidar("RobotisLds01");
  webots::InertialUnit *wb_imu = robot->getInertialUnit("imu");

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(wb_lidar, &nh);
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(wb_imu, &nh);

  while (robot->step(step_size) != -1) {
    lidar.publishLaserScan();
    imu.publishImuQuaternion();
  }

  // Clean up
  delete robot;
  return 0;
}
