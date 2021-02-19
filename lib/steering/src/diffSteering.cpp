#include "ros/ros.h"
#include "steering/diffSteering.hpp"
#include "geometry_msgs/Twist.h"
#include "webots/Motor.hpp"
#include "webots/Keyboard.hpp"

using namespace AutomatED;

DiffSteering::DiffSteering(webots::Motor motors[4], ros::NodeHandle *ros_handle){
  wheels = motors;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("diffSteering/wheel_separation", wheel_separation, 0.6);
  nh->param("diffSteering/wheel_radius", wheel_radius, 0.12);

  // Create Subscriber
  cmd_vel_sub = nh.subscribe("cmd_vel", 1, velocityCallback);

  // Turn on motors
  turnOnMotors();

}

DiffSteering::~DiffSteering(){
  // Clean up
  cmd_vel_sub.shutdown()
  shutDownMotors()
}

void velocityCallback(const geometry_msgs::Twist& cmd){
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

void keyboardInput() {
  double speed = 2.0;
  webots::Keyboard keyboard;
  int key = keyboard.getKey();
  if (key >= 0) {
    switch (key) {
      case keyboard.UP:
        wheels[0]->setVelocity(speed);
        wheels[2]->setVelocity(speed);
        wheels[1]->setVelocity(speed);
        wheels[3]->setVelocity(speed);
        break;
      case keyboard.DOWN:
        wheels[0]->setVelocity(-speed);
        wheels[2]->setVelocity(-speed);
        wheels[1]->setVelocity(-speed);
        wheels[3]->setVelocity(-speed);
        break;
      case keyboard.RIGHT:
        wheels[0]->setVelocity(speed);
        wheels[2]->setVelocity(speed);
        wheels[1]->setVelocity(-speed);
        wheels[3]->setVelocity(-speed);
        break;
      case keyboard.LEFT:
        wheels[0]->setVelocity(-speed);
        wheels[2]->setVelocity(-speed);
        wheels[1]->setVelocity(speed);
        wheels[3]->setVelocity(speed);
        break;
}
