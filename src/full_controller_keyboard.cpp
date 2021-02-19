#include "ros/ros.h"
#include "sensors/gps.hpp"
#include "webots/GPS.hpp"
#include "sensors/lidar.hpp"
#include "webots/Lidar.hpp"
#include "sensors/inertialUnit.hpp"
#include "webots/InertialUnit.hpp"
#include "steering/diffSteering.hpp"
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
  std::string gps_name;
  nh.param<std::string>("gps/name", gps_name, "gps");
  int step_size;
  nh.param("step_size", step_size, 32);

  // Set up Webots
  webots::Robot *robot = new webots::Robot();

  // Get webot devices
  webots::Lidar *wb_lidar = robot->getLidar(lidar_name);
  webots::GPS *wb_gps = robot->getGPS(gps_name);
  webots::InertialUnit *wb_imu = robot->getInertialUnit(imu_name);
  webots::Motor *motors[4] = {
    robot->getMotor("front_left_wheel"),
    robot->getMotor("front_right_wheel"),
    robot->getMotor("rear_left_wheel"),
    robot->getMotor("rear_right_wheel")
  };

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(wb_lidar, &nh);
  AutomatED::GPS gps = AutomatED::GPS(wb_gps, &nh);
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(wb_imu, &nh);

  // Instantiate steering
  AutomatED::DiffSteering diffSteering = AutomatED::DiffSteering(motors, &nh);

  while (robot->step(step_size) != -1) {
    lidar.publishLaserScan();
    gps.publishGPSCoordinate();
    gps.publishGPSSpeed();
    imu.publishImuQuaternion();
    diffSteering.keyboardInput();
  }

  // Clean up
  delete robot;
  return 0;
}
