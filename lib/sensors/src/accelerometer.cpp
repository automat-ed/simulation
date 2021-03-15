#include "sensors/accelerometer.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Accelerometer.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

Accelerometer::Accelerometer(webots::Supervisor *webots_supervisor,
                             ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("accelerometer/name", accelerometer_name,
                         "accelerometer");
  nh->param<std::string>("accelerometer/frame_id", frame_id, "accelerometer");
  nh->param("accelerometer/sampling_period", sampling_period, 32);
  nh->param<std::string>("accelerometer/ground_truth_topic", ground_truth_topic,
                         "/accelerometer/ground_truth");
  nh->param<std::string>("accelerometer/noise_topic", noise_topic,
                         "/accelerometer/data");
  nh->param("accelerometer/noise_mean", noise_mean, 0.0);
  nh->param("accelerometer/noise_std", noise_std, 0.017);
  nh->param("accelerometer/bias_mean", bias_mean, 0.1);
  nh->param("accelerometer/bias_std", bias_std, 0.001);
  nh->param<int>("accelerometer/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Imu>(ground_truth_topic, 1);
  noise_pub = nh->advertise<sensor_msgs::Imu>(noise_topic, 1);

  // Setup accelerometer device
  accelerometer = wb->getAccelerometer(accelerometer_name);
  accelerometer->enable(sampling_period);

  // Publish tf
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> d{bias_mean, bias_std};
  bias = d(*gen);
}

Accelerometer::~Accelerometer()
{
  noise_pub.shutdown();
  ground_truth_pub.shutdown();
  accelerometer->disable();
}

void Accelerometer::publishAccelerometer()
{
  // Read from sensor
  const double *reading = accelerometer->getValues();

  // Extract data
  double webots_x = reading[0];
  double webots_y = reading[1];
  double webots_z = reading[2];

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;

  // Unset values are set to 0 by default
  gt.orientation_covariance[0] = -1.0; // means no orientation information
  gt.angular_velocity_covariance[0] = -1.0; // no angular_velocity information
  gt.linear_acceleration.x = webots_x;
  gt.linear_acceleration.y = 0;
  gt.linear_acceleration.z = 0;

  for (int i = 0; i < 9; i++) // means "covariance unknown"
    gt.linear_acceleration_covariance[i] = 0;

  ground_truth_pub.publish(gt);
}

void Accelerometer::publishTF()
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

double Accelerometer::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
