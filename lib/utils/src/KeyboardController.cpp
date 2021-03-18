#include "utils/KeyboardController.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "webots/Keyboard.hpp"

using namespace AutomatED;

KeyboardController::KeyboardController(webots::Supervisor *webots_supervisor,
                                       ros::NodeHandle *ros_handle) {
  ROS_INFO("Starting C++ keyboard controller...");

  // ROS Node Handle
  nh_ = ros_handle;

  // ROS Parameters
  nh_->param("keyboard/avel_scale", a_scale_, 0.3);
  nh_->param("keyboard/lvel_scale", l_scale_, 0.3);
  nh_->param("keyboard/time_step", time_step_, 32);
  nh_->param("keyboard/frequency", frequency_, 50);

  // ROS Publisher
  cmd_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // Enable Keyboard
  keyboard_ = new webots::Keyboard();
  keyboard_->enable(time_step_);
  prev_key = -1;
}

KeyboardController::~KeyboardController() {
  cmd_pub_.shutdown();
  keyboard_->disable();
}

void KeyboardController::keyLoop() {
  // Get key value
  int key = keyboard_->getKey();

  if (prev_key != key) {
    // Reset control values
    linear_ = 0;
    angular_ = 0;

    // Match key
    switch (key) {
    case webots::Keyboard::UP:
      ROS_DEBUG("Up key pressed");
      linear_ = 1.0;
      break;
    case webots::Keyboard::DOWN:
      ROS_DEBUG("Down key pressed");
      linear_ = -1.0;
      break;
    case webots::Keyboard::RIGHT:
      ROS_DEBUG("Right key pressed");
      angular_ = -1.0;
      break;
    case webots::Keyboard::LEFT:
      ROS_DEBUG("Left key pressed");
      angular_ = 1.0;
      break;
    }

    // Update prev_key
    prev_key = key;

    // Create control message
    geometry_msgs::Twist msg;
    msg.angular.z = a_scale_ * angular_;
    msg.linear.x = l_scale_ * linear_;
    cmd_pub_.publish(msg);
  }
}