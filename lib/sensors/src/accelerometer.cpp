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

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;

  // Unset values are set to 0 by default
  gt.orientation_covariance[0] = -1.0; // means no orientation information
  gt.angular_velocity_covariance[0] = -1.0; // no angular_velocity information
  gt.linear_acceleration.x = reading[0];
  gt.linear_acceleration.y = reading[1];
  gt.linear_acceleration.z = reading[2];

  for (int i = 0; i < 9; i++) // means "covariance unknown"
    gt.linear_acceleration_covariance[i] = 0;

  ground_truth_pub.publish(gt);

  // Publish noisy data
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  // Unset values are set to 0 by default
  msg.orientation_covariance[0] = -1;      // means no orientation information
  msg.angular_velocity_covariance[0] = -1; // no angular_velocity information
  msg.linear_acceleration.x = reading[0] + bias + gaussianNoise();
  msg.linear_acceleration.y = reading[1] + bias + gaussianNoise();
  msg.linear_acceleration.z = reading[2] + bias + gaussianNoise();

  // Populate variance along the diagonal
  msg.linear_acceleration_covariance[0] = std::pow(bias + noise_mean + noise_std, 2);
  msg.linear_acceleration_covariance[4] = std::pow(bias + noise_mean + noise_std, 2);
  msg.linear_acceleration_covariance[8] = std::pow(bias + noise_mean + noise_std, 2);

  noise_pub.publish(msg);
}

void Accelerometer::publishTF()
{
  // Get accelerometer node
  webots::Node *accelerometer_node = wb->getFromDevice(accelerometer);

  // Get accelerometer translation
  webots::Field *accelerometer_translation_field =
      accelerometer_node->getField("translation");
  const double *accelerometer_translation =
      accelerometer_translation_field->getSFVec3f();

  ROS_DEBUG("Accelerometer translation: [%f, %f, %f]",
            accelerometer_translation[0],
            accelerometer_translation[1],
            accelerometer_translation[2]);

  // Get accelerometer rotation
  webots::Field *accelerometer_rotation_field =
      accelerometer_node->getField("rotation");
  const double *accelerometer_rotation =
      accelerometer_rotation_field->getSFRotation();

  ROS_DEBUG("Accelerometer rotation: [%f, %f, %f, %f]",
            accelerometer_rotation[0],
            accelerometer_rotation[1],
            accelerometer_rotation[2],
            accelerometer_rotation[3]);

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = frame_id;

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = accelerometer_translation[0];
  msg.transform.translation.y = -1 * accelerometer_translation[2];
  msg.transform.translation.z = accelerometer_translation[1];

  tf2::Quaternion rot;
  rot.setRotation({accelerometer_rotation[0],
                   accelerometer_rotation[1],
                   accelerometer_rotation[2]}, accelerometer_rotation[3]);

  tf2::Quaternion ros_to_webots;
  ros_to_webots.setRPY(1.5707, 0, 0);

  tf2::Quaternion quat = ros_to_webots * rot;
  quat.normalize();
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}

double Accelerometer::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
