#include "sensors/wheelOdometry.hpp"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "ros/ros.h"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

WheelOdometry::WheelOdometry(webots::Supervisor *webots_supervisor,
                             ros::NodeHandle *ros_handle)
{

  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("wheel_odom/frame_id", frame_id, "base_link");
  nh->param("wheel_odom/sampling_period", sampling_period, 32);
  nh->param<std::string>("wheel_odom/ground_truth_topic", ground_truth_topic,
                         "/wheel_odom/ground_truth");
  nh->param<std::string>("wheel_odom/noise_topic", noise_topic,
                         "/wheel_odom/data");
  nh->param("wheel_separation", wheel_separation, 0.6);
  nh->param("wheel_radius", wheel_radius, 0.12);
  nh->param("wheel_odom/linear_mean", linear_mean, 0.0);
  nh->param("wheel_odom/linear_std", linear_std, 0.017);
  nh->param("wheel_odom/linear_bias_mean", linear_bias_mean, 0.1);
  nh->param("wheel_odom/linear_bias_std", linear_bias_std, 0.001);
  nh->param("wheel_odom/angular_mean", angular_mean, 0.0);
  nh->param("wheel_odom/angular_std", angular_std, 0.0002);
  nh->param("wheel_odom/angular_bias_mean", angular_bias_mean, 0.0000075);
  nh->param("wheel_odom/angular_bias_std", angular_bias_std, 0.0000008);
  nh->param<int>("wheel_odom/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub =
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>(ground_truth_topic, 1);
  noise_pub = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>(noise_topic, 1);

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> dl{linear_bias_mean, linear_bias_std};
  linear_bias = dl(*gen);
  std::normal_distribution<double> da{angular_bias_mean, angular_bias_std};
  angular_bias = da(*gen);
}

WheelOdometry::~WheelOdometry()
{
  ground_truth_pub.shutdown();
  noise_pub.shutdown();
}

void WheelOdometry::publishWheelOdometry()
{
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

  // Publish ground truth
  geometry_msgs::TwistWithCovarianceStamped gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;
  gt.twist.twist.linear.x = robot_vel;
  gt.twist.twist.angular.z = robot_rvel;
  ground_truth_pub.publish(gt);

  // Publish noisy data
  geometry_msgs::TwistWithCovarianceStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.twist.twist.linear.x = robot_vel + linear_bias + linearGaussianNoise();
  msg.twist.twist.angular.z = robot_rvel + angular_bias + angularGaussianNoise();
  msg.twist.covariance[0] = std::pow(linear_bias + linear_mean + linear_std, 2);
  msg.twist.covariance[35] = std::pow(angular_bias + angular_mean + angular_std, 2);
  noise_pub.publish(msg);
}

void WheelOdometry::getLocalRotationalVelocity(webots::Node *solid,
                                               double *rvel_local)
{
  const double *vel = solid->getVelocity();
  const double *orientation = solid->getOrientation();

  double orientation_t[9];
  WheelOdometry::transposeOrientation(orientation, orientation_t);
  WheelOdometry::transformVelocity(orientation_t, vel, rvel_local);
}

void WheelOdometry::transposeOrientation(const double *matrix,
                                         double *matrix_t)
{
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
                                      const double *vector, double *new_vec)
{
  new_vec[0] = (matrix[0] * vector[3]) + (matrix[1] * vector[4]) +
               (matrix[2] * vector[5]);
  new_vec[1] = (matrix[3] * vector[3]) + (matrix[4] * vector[4]) +
               (matrix[5] * vector[2]);
  new_vec[2] = (matrix[6] * vector[3]) + (matrix[7] * vector[4]) +
               (matrix[8] * vector[5]);
}

double WheelOdometry::linearGaussianNoise()
{
  std::normal_distribution<double> d{linear_mean, linear_std};
  return d(*gen);
}

double WheelOdometry::angularGaussianNoise()
{
  std::normal_distribution<double> d{angular_mean, angular_std};
  return d(*gen);
}
