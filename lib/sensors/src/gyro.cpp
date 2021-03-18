#include "sensors/gyro.hpp"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Gyro.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

Gyro::Gyro(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("gyro/name", gyro_name, "gyro");
  nh->param<std::string>("gyro/frame_id", frame_id, "gyro");
  nh->param("gyro/sampling_period", sampling_period, 32);
  nh->param("gyro/noise_mean", noise_mean, 0.0);
  nh->param("gyro/noise_std", noise_std, 0.0002);
  nh->param<int>("gyro/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Imu>("/gyro/ground_truth", 1);
  noise_pub = nh->advertise<sensor_msgs::Imu>("/gyro/data", 1);

  // Setup IMU device
  gyro = wb->getGyro(gyro_name);
  gyro->enable(sampling_period);

  // Publish tf
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};
}

Gyro::~Gyro()
{
  noise_pub.shutdown();
  ground_truth_pub.shutdown();
  gyro->disable();
}

void Gyro::publishGyro()
{
  // Get data from Gyro
  const double *reading = gyro->getValues();

  // Get time
  ros::Time current_time = ros::Time::now();

  // Extract data
  double webots_x = reading[0];
  double webots_y = reading[1];
  double webots_z = reading[2];

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = current_time;
  gt.header.frame_id = frame_id;
  // Unset values are set to 0 by default
  gt.orientation_covariance[0] = -1.0; // means no orientation information
  gt.angular_velocity.x = 0;
  gt.angular_velocity.y = 0;
  gt.angular_velocity.z = webots_y;
  gt.linear_acceleration_covariance[0] = -1.0; // means no lin acc information

  ground_truth_pub.publish(gt);

  // Publish noisy data
  sensor_msgs::Imu msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = frame_id;
  // Unset values are set to 0 by default
  msg.orientation_covariance[0] = -1.0; // means no orientation information
  msg.angular_velocity.x = 0;
  msg.angular_velocity.y = 0;
  msg.angular_velocity.z = webots_y + gaussianNoise();
  msg.angular_velocity_covariance[8] = std::pow(noise_mean + noise_std, 2);
  msg.linear_acceleration_covariance[0] = -1.0; // means no lin acc information

  noise_pub.publish(msg);
}

void Gyro::publishTF()
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

double Gyro::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
