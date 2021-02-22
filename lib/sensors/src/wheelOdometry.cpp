#include "sensors/wheelOdometry.hpp"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "webots/Supervisor.hpp"

using namespace AutomatED;

WheelOdometry::WheelOdometry(webots::Supervisor *webots_supervisor,
                             ros::NodeHandle *ros_handle) {

  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("wheel_odometry/sampling_period", sampling_period, 32);
  nh->param<std::string>("wheel_odometry/odom_topic",
                         wheel_odometry_twist_topic, "wheel_odom");
  nh->param("wheel_separation", wheel_separation, 0.6);
  nh->param("wheel_radius", wheel_radius, 0.12);

  // Create publishers
  wheel_odometry_pub =
      nh->advertise<geometry_msgs::TwistStamped>(wheel_odometry_twist_topic, 1);
}

WheelOdometry::~WheelOdometry() { wheel_odometry_pub.shutdown(); }

void WheelOdometry::publishWheelOdometry() {
  // Get wheels from webots
  webots::Node *fl_joint = wb->getFromDef("FRONT_LEFT_SOLID");
  webots::Node *fr_joint = wb->getFromDef("FRONT_RIGHT_SOLID");
  webots::Node *rl_joint = wb->getFromDef("REAR_LEFT_SOLID");
  webots::Node *rr_joint = wb->getFromDef("REAR_RIGHT_SOLID");

  // Get rotation velocity in wheel coordinate frame
  double fl_rvel[3];
  WheelOdometry::getLocalRotationalVelocity(fl_joint, fl_rvel);
  double fr_rvel[3];
  WheelOdometry::getLocalRotationalVelocity(fr_joint, fr_rvel);
  double rl_rvel[3];
  WheelOdometry::getLocalRotationalVelocity(rl_joint, rl_rvel);
  double rr_rvel[3];
  WheelOdometry::getLocalRotationalVelocity(rr_joint, rr_rvel);

  // Take average of both sides
  const double lvel_avg = (fl_rvel[2] + rl_rvel[2]) / 2;
  const double rvel_avg = (fr_rvel[2] + rr_rvel[2]) / 2;

  // Calculate linear and rotational velocity of robot
  const double robot_vel =
      (lvel_avg * wheel_radius + rvel_avg * wheel_radius) / 2;
  const double robot_rvel =
      (rvel_avg * wheel_radius - lvel_avg * wheel_radius) / wheel_separation;

  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.twist.linear.x = robot_vel;
  msg.twist.linear.y = 0;
  msg.twist.linear.z = 0;
  msg.twist.angular.x = 0;
  msg.twist.angular.y = 0;
  msg.twist.angular.z = robot_rvel;

  // Publish message
  wheel_odometry_pub.publish(msg);
}

void WheelOdometry::getLocalRotationalVelocity(webots::Node *solid,
                                               double *rvel_local) {
  const double *vel = solid->getVelocity();
  const double *orientation = solid->getOrientation();

  double orientation_t[9];
  WheelOdometry::transposeOrientation(orientation, orientation_t);
  WheelOdometry::transformVelocity(orientation_t, vel, rvel_local);
}

void WheelOdometry::transposeOrientation(const double *matrix,
                                         double *matrix_t) {
  matrix_t[0] = matrix[0];
  matrix_t[1] = matrix[3];
  matrix_t[2] = matrix[6];
  matrix_t[3] = matrix[1];
  matrix_t[4] = matrix[4];
  matrix_t[5] = matrix[7];
  matrix_t[6] = matrix[2];
  matrix_t[7] = matrix[5];
  matrix_t[8] = matrix[8];
}

void WheelOdometry::transformVelocity(const double *matrix,
                                      const double *vector, double *new_vec) {
  new_vec[0] = (matrix[0] * vector[3]) + (matrix[1] * vector[4]) +
               (matrix[2] * vector[5]);
  new_vec[1] = (matrix[3] * vector[3]) + (matrix[4] * vector[4]) +
               (matrix[5] * vector[2]);
  new_vec[2] = (matrix[6] * vector[3]) + (matrix[7] * vector[4]) +
               (matrix[8] * vector[5]);
}