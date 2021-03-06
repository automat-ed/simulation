#include "sensors/InertialUnit.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/InertialUnit.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

InertialUnit::InertialUnit(webots::Supervisor *webots_supervisor,
                           ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("imu/name", imu_name, "imu");
  nh->param<std::string>("imu/frame_id", frame_id, "imu");
  nh->param("imu/sampling_period", sampling_period, 32);
  nh->param("imu/noise_mean", noise_mean, 0.0);
  nh->param("imu/noise_std", noise_std, 0.02);
  nh->param<int>("imu/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Imu>("/imu/ground_truth", 1);
  noise_pub = nh->advertise<sensor_msgs::Imu>("/imu/data", 1);

  // Setup IMU device
  imu = wb->getInertialUnit(imu_name);
  imu->enable(sampling_period);

  // Publish tf
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};
}

InertialUnit::~InertialUnit()
{
  ground_truth_pub.shutdown();
  noise_pub.shutdown();
  imu->disable();
}

void InertialUnit::publishImuQuaternion()
{
  // Get sensor reading
  const double *reading = imu->getRollPitchYaw();

  // Get time
  ros::Time current_time = ros::Time::now();

  // Extract data
  double webots_yaw = reading[2];

  // Shift reading to get 0 when facing East
  webots_yaw += 1.5708;

  // Convert to Quaternion
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, webots_yaw);
  quaternion.normalize();

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = current_time;
  gt.header.frame_id = frame_id;
  gt.orientation.x = quaternion.getX();
  gt.orientation.y = quaternion.getY();
  gt.orientation.z = quaternion.getZ();
  gt.orientation.w = quaternion.getW();
  gt.angular_velocity_covariance[0] = -1;      // no information
  gt.linear_acceleration_covariance[0] = -1.0; // no information
  ground_truth_pub.publish(gt);

  // Convert to Quaternion (with noise)
  tf2::Quaternion noisy_quaternion;
  noisy_quaternion.setRPY(0, 0, webots_yaw + gaussianNoise());
  noisy_quaternion.normalize();

  // Publish noisy data
  sensor_msgs::Imu msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = frame_id;
  msg.orientation.x = noisy_quaternion.getX();
  msg.orientation.y = noisy_quaternion.getY();
  msg.orientation.z = noisy_quaternion.getZ();
  msg.orientation.w = noisy_quaternion.getW();
  msg.orientation_covariance[8] = std::pow(noise_mean + noise_std, 2);
  msg.angular_velocity_covariance[0] = -1;      // no information
  msg.linear_acceleration_covariance[0] = -1.0; // no information
  noise_pub.publish(msg);
}

void InertialUnit::publishTF()
{
  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = frame_id;

  msg.transform.translation.x = 0;
  msg.transform.translation.y = 0;
  msg.transform.translation.z = 0;

  msg.transform.rotation.x = 0;
  msg.transform.rotation.y = 0;
  msg.transform.rotation.z = 0;
  msg.transform.rotation.w = 1;

  static_broadcaster.sendTransform(msg);
}

double InertialUnit::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
