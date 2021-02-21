#include "steering/diffSteering.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "webots/Motor.hpp"

using namespace AutomatED;

DiffSteering::DiffSteering(std::vector<webots::Motor *> motors,
                           ros::NodeHandle *ros_handle) {
  wheels = motors;
  nh = ros_handle;
  wheel_count = wheels.size();

  // Get ROS parameters
  nh->param("wheel_separation", wheel_separation, 0.6);
  nh->param("wheel_radius", wheel_radius, 0.12);

  // Create Subscriber
  cmd_vel_sub =
      nh->subscribe("/cmd_vel", 1, &DiffSteering::velocityCallback, this);

  // Turn on motors
  for (int i = 0; i < wheel_count; i++) {
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
}

DiffSteering::~DiffSteering() {
  // Clean up
  cmd_vel_sub.shutdown();
  stopMotors();
}

void DiffSteering::velocityCallback(const geometry_msgs::TwistConstPtr &cmd) {
  double linear_vel = cmd->linear.x;
  double angular_vel = cmd->angular.z;

  const double vel_left =
      (linear_vel - angular_vel * wheel_separation / 2.0) / wheel_radius;
  const double vel_right =
      (linear_vel + angular_vel * wheel_separation / 2.0) / wheel_radius;

  // Set velocity to left wheels
  wheels[0]->setVelocity(vel_left);
  wheels[2]->setVelocity(vel_left);
  // Set velocity to right wheels
  wheels[1]->setVelocity(vel_right);
  wheels[3]->setVelocity(vel_right);
}

void DiffSteering::stopMotors() {
  for (int i = 0; i < wheel_count; i++) {
    wheels[i]->setVelocity(0.0);
  }
}
