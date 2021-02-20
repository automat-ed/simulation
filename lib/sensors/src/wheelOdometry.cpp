#include "ros/ros.h"
#include "sensors/wheelOdometry.hpp"
#include "webots/Supervisor.hpp"
#include "geometry_msgs/TwistStamped.h"


using namespace AutomatED;

WheelOdometry::WheelOdometry(webots::Supervisor *webots_supervisor,
                             ros::NodeHandle *ros_handle) {

  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("wheel_odometry/sampling_period", sampling_period, 32);
  nh->param<std::string>("wheel_odometry/twist_topic", wheel_odometry_twist_topic, 
                         "wheel_odometry_twist");
  nh->param("wheel_odometry/wheel_separation", wheel_separation, 0.6);
  nh->param("wheel_odometry/wheel_radius", wheel_radius, 0.12);


  // Create publishers
  wheel_odometry_pub = 
      nh->advertise<geometry_msgs::TwistStamped>(wheel_odometry_twist_topic, 1);



  // TODO create an array of wheel names in odom file and iterate over that
  // Get wheels
  wheels[0] = wb->getMotor("front_left_wheel");
  wheels[1] = wb->getMotor("front_right_wheel");
  wheels[2] = wb->getMotor("rear_left_wheel");
  wheels[3] = wb->getMotor("rear_right_wheel");

}


WheelOdometry::~WheelOdometry() {
  wheel_odometry_pub.shutdown();
}


void WheelOdometry::publishWheelOdometry() {
  webots::Node *front_left_node = wb->getFromDevice(wheels[0]);
  webots::Node *front_right_node = wb->getFromDevice(wheels[0]);
  webots::Node *rear_left_node = wb->getFromDevice(wheels[0]);
  webots::Node *rear_right_node = wb->getFromDevice(wheels[0]);

  // Get Linear and Angular velocities
  const double *vel0 = front_left_node->getVelocity();
  const double *vel1 = front_right_node->getVelocity();
  const double *vel2 = rear_left_node->getVelocity();
  const double *vel3 = rear_right_node->getVelocity();

  const double avg_vel_left  = (vel0[0] + vel2[0]) / 2;
  const double avg_vel_right = (vel1[0] + vel3[0]) / 2;

  const double linear_vel_robot = 
      wheel_radius * avg_vel_left  + (wheel_separation * avg_vel_right - wheel_separation * avg_vel_left);
  const double angular_vel_robot = 
      (wheel_radius * avg_vel_right - wheel_radius * avg_vel_left) / wheel_separation;

  geometry_msgs::TwistStamped msg; 
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "wheel_odometry";
  msg.twist.linear.x = linear_vel_robot;
  msg.twist.linear.y = 0;
  msg.twist.linear.z = 0;
  msg.twist.angular.x = 0;
  msg.twist.angular.y = 0;
  msg.twist.angular.z = angular_vel_robot;

  wheel_odometry_pub.publish(msg);

}
