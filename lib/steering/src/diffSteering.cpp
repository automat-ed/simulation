#include "steering/diffSteering.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "ros/ros.h"
#include "webots/Motor.hpp"

using namespace AutomatED;

DiffSteering::DiffSteering(std::vector<webots::Motor *> motors,
                           ros::NodeHandle *ros_handle)
{
  wheels = motors;
  nh = ros_handle;
  wheel_count = wheels.size();

  // Get ROS parameters
  nh->param<std::string>("wheel_odom/frame_id", frame_id, "base_link");
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

  // Create Subscriber
  cmd_vel_sub =
      nh->subscribe("/cmd_vel", 1, &DiffSteering::velocityCallback, this);

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> dl{linear_bias_mean, linear_bias_std};
  linear_bias = dl(*gen);
  std::normal_distribution<double> da{angular_bias_mean, angular_bias_std};
  angular_bias = da(*gen);

  // Turn on motors
  for (int i = 0; i < wheel_count; i++)
  {
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
}

DiffSteering::~DiffSteering()
{
  // Clean up
  cmd_vel_sub.shutdown();
  ground_truth_pub.shutdown();
  noise_pub.shutdown();
  stopMotors();
}

void DiffSteering::velocityCallback(const geometry_msgs::TwistConstPtr &cmd)
{
  linear_vel = cmd->linear.x;
  angular_vel = cmd->angular.z;

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

void DiffSteering::publish()
{
  // Publish ground truth wheel odometry
  geometry_msgs::TwistWithCovarianceStamped gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;
  gt.twist.twist.linear.x = linear_vel;
  gt.twist.twist.angular.z = angular_vel;
  ground_truth_pub.publish(gt);

  // Publish noisy wheel odometry data
  geometry_msgs::TwistWithCovarianceStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.twist.twist.linear.x = linear_vel + linear_bias + linearGaussianNoise();
  msg.twist.twist.angular.z = angular_vel + angular_bias + angularGaussianNoise();
  msg.twist.covariance[0] = std::pow(linear_bias + linear_mean + linear_std, 2);
  msg.twist.covariance[35] = std::pow(angular_bias + angular_mean + angular_std, 2);
  noise_pub.publish(msg);
}

void DiffSteering::stopMotors()
{
  for (int i = 0; i < wheel_count; i++)
  {
    wheels[i]->setVelocity(0.0);
  }
}

double DiffSteering::linearGaussianNoise()
{
  std::normal_distribution<double> d{linear_mean, linear_std};
  return d(*gen);
}

double DiffSteering::angularGaussianNoise()
{
  std::normal_distribution<double> d{angular_mean, angular_std};
  return d(*gen);
}
