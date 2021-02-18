#include "ros/ros.h"
#include "sensors/gps.hpp"
#include "sensors/inertialUnit.hpp"
#include "sensors/lidar.hpp"
#include "webots/GPS.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/Lidar.hpp"
#include "webots/Motor.hpp"
#include "webots/Supervisor.hpp"

int main(int argc, char **argv) {
  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters

  std::string imu_name;
  nh.param<std::string>("imu/name", imu_name, "imu");
  std::string gps_name;
  nh.param<std::string>("gps/name", gps_name, "gps");
  int step_size;
  nh.param("step_size", step_size, 32);

  // Set up Webots
  webots::Supervisor *supervisor = new webots::Supervisor();

  // Get webot devices
  webots::GPS *wb_gps = supervisor->getGPS(gps_name);
  webots::InertialUnit *wb_imu = supervisor->getInertialUnit(imu_name);

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(supervisor, &nh);
  AutomatED::GPS gps = AutomatED::GPS(wb_gps, &nh);
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(wb_imu, &nh);

  while (supervisor->step(step_size) != -1) {
    lidar.publishLaserScan();
    gps.publishGPSCoordinate();
    gps.publishGPSSpeed();
    imu.publishImuQuaternion();
  }

  // Clean up
  delete supervisor;
  return 0;
}
