#include "ros/ros.h"
#include "steering/diffSteering.hpp"
#include "geometry_msgs/Twist.h"
#include "webots/Motor.hpp"

using namespace AutomatED;

DiffSteering::DiffSteering(webots::Motor motors[4], ros::NodeHandle *ros_handle){
  wheels = motors;
  nh = ros_handle;
  wheel_separation = 0.6;
  wheel_radius = 0.12;

  // Create Subscriber
  cmd_vel_sub = nh.subscribe("cmd_vel", 1000, subscribeVelocity);

  // Turn on motors
  turnOnMotors();

}

DiffSteering::~DiffSteering(){
  //Clean up
  cmd_vel_sub.shutdown()
  shutDownMotors()
}

void subscribeVelocity(const geometry_msgs::Twist& cmd){
  double linear_vel = cmd.linear.x;
  double angular_vel = cmd.angluar.z;
  
  const double vel_left  = 
          (linear_vel - angular_vel * wheel_separation / 2.0)/wheel_radius;
  const double vel_right =
          (linear_vel + angular_vel * wheel_separation / 2.0)/wheel_radius;

  // Set velocity to left wheels
  wheels[0]->setVelocity(vel_left);
  wheels[2]->setVelocity(vel_left);
  // Set velocity to right wheels
  wheels[1]->setVelocity(vel_right);
  wheels[3]->setVelocity(vel_right);
}

void turnOnMotors() {
  for (int i = 0; i < std::size(wheels); i++) {
    wheels[i]->setPosition(std::numeric_limits<double>::infinity);
  } 
}

void shutDownMotors() {
  for (int i = 0; i < std::size(wheels); i++) {
    wheels[i]->setVelocity(0.0);
  }
  
}
