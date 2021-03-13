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
  nh->param<std::string>("gyro/ground_truth_topic", ground_truth_topic,
                         "/gyro/ground_truth");
  nh->param<std::string>("gyro/noise_topic", noise_topic, "/gyro/data");
  nh->param("gyro/noise_mean", noise_mean, 0.0);
  nh->param("gyro/noise_std", noise_std, 0.0002);
  nh->param("gyro/bias_mean", bias_mean, 0.0000075);
  nh->param("gyro/bias_std", bias_std, 0.0000008);
  nh->param<int>("gyro/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Imu>(ground_truth_topic, 1);
  noise_pub = nh->advertise<sensor_msgs::Imu>(noise_topic, 1);

  // Setup IMU device
  gyro = wb->getGyro(gyro_name);
  gyro->enable(sampling_period);

  // Publish tf
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> d{bias_mean, bias_std};
  bias = d(*gen);
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

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;

  // Unset values are set to 0 by default
  gt.orientation_covariance[0] = -1.0; // means no orientation information
  gt.angular_velocity.x = reading[0];
  gt.angular_velocity.y = reading[1];
  gt.angular_velocity.z = reading[2];
  gt.linear_acceleration_covariance[0] = -1.0; // means no information

  ground_truth_pub.publish(gt);

  // Publish noisy data
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  // Unset values are set to 0 by default
  msg.orientation_covariance[0] = -1.0; // means no orientation information
  msg.angular_velocity.x = reading[0] + bias + gaussianNoise();
  msg.angular_velocity.y = reading[1] + bias + gaussianNoise();
  msg.angular_velocity.z = reading[2] + bias + gaussianNoise();
  // Populate variance along the diagonal
  msg.angular_velocity_covariance[0] = std::pow(bias + noise_mean + noise_std, 2);
  msg.angular_velocity_covariance[4] = std::pow(bias + noise_mean + noise_std, 2);
  msg.angular_velocity_covariance[8] = std::pow(bias + noise_mean + noise_std, 2);
  msg.linear_acceleration_covariance[0] = -1.0; // means no lin acc information

  noise_pub.publish(msg);
}

void Gyro::publishTF()
{
  // Get gyro node
  webots::Node *gyro_node = wb->getFromDevice(gyro);

  // Get gyro translation
  webots::Field *gyro_translation_field = gyro_node->getField("translation");
  const double *gyro_translation = gyro_translation_field->getSFVec3f();

  ROS_DEBUG("Gyro translation: [%f, %f, %f]",
            gyro_translation[0],
            gyro_translation[1],
            gyro_translation[2]);

  // Get gyro rotation
  webots::Field *gyro_rotation_field = gyro_node->getField("rotation");
  const double *gyro_rotation = gyro_rotation_field->getSFRotation();

  ROS_DEBUG("Gyro rotation: [%f, %f, %f, %f]",
            gyro_rotation[0],
            gyro_rotation[1],
            gyro_rotation[2],
            gyro_rotation[3]);

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = frame_id;

  // Translate from ROS to Webots coordinates
  msg.transform.translation.x = gyro_translation[0];
  msg.transform.translation.y = -1 * gyro_translation[2];
  msg.transform.translation.z = gyro_translation[1];

  tf2::Quaternion rot;
  rot.setRotation({gyro_rotation[0],
                   gyro_rotation[1],
                   gyro_rotation[2]}, gyro_rotation[3]);

  tf2::Quaternion ros_to_webots;
  ros_to_webots.setRPY(1.5708, 0, 0);

  tf2::Quaternion quat = ros_to_webots * rot;
  quat.normalize();
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}

double Gyro::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
