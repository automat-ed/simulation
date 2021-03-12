#include "sensors/inertialUnit.hpp"
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
  nh->param<std::string>("imu/ground_truth_topic", ground_truth_topic,
                         "/imu/ground_truth");
  nh->param<std::string>("imu/noise_topic", noise_topic, "/imu/data");
  nh->param("imu/noise_mean", noise_mean, 0.0);
  nh->param("imu/noise_std", noise_std, 0.02);
  nh->param("imu/bias_mean", bias_mean, 0.005);
  nh->param("imu/bias_std", bias_std, 0.001);
  nh->param<int>("imu/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Imu>(ground_truth_topic, 1);
  noise_pub = nh->advertise<sensor_msgs::Imu>(noise_topic, 1);

  // Setup IMU device
  imu = wb->getInertialUnit(imu_name);
  imu->enable(sampling_period);

  // Publish tf
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> d{bias_mean, bias_std};
  bias = d(*gen);
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
  const double *reading = imu->getQuaternion();

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;
  gt.orientation.x = reading[0];
  gt.orientation.y = reading[1];
  gt.orientation.z = reading[2];
  gt.orientation.w = reading[3];
  gt.angular_velocity_covariance[0] = -1;      // no information
  gt.linear_acceleration_covariance[0] = -1.0; // no information
  ground_truth_pub.publish(gt);

  // Publish noisy data
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.orientation.x = reading[0] + bias + gaussianNoise();
  msg.orientation.y = reading[1] + bias + gaussianNoise();
  msg.orientation.z = reading[2] + bias + gaussianNoise();
  msg.orientation.w = reading[3] + bias + gaussianNoise();
  // Populate variance along the diagonal
  msg.orientation_covariance[0] = std::pow(bias + noise_mean + noise_std, 2);
  msg.orientation_covariance[4] = std::pow(bias + noise_mean + noise_std, 2);
  msg.orientation_covariance[8] = std::pow(bias + noise_mean + noise_std, 2);
  msg.angular_velocity_covariance[0] = -1;      // no information
  msg.linear_acceleration_covariance[0] = -1.0; // no information

  noise_pub.publish(msg);
}

void InertialUnit::publishTF()
{
  // Get imu node
  webots::Node *imu_node = wb->getFromDevice(imu);

  // Get imu translation
  webots::Field *imu_translation_field = imu_node->getField("translation");
  const double *imu_translation = imu_translation_field->getSFVec3f();

  // Get imu rotation
  webots::Field *imu_rotation_field = imu_node->getField("rotation");
  const double *imu_rotation = imu_rotation_field->getSFRotation();

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = frame_id;

  // Translate from ROS to Webots coordinates
  msg.transform.translation.x = imu_translation[0];
  msg.transform.translation.y = -1 * imu_translation[2];
  msg.transform.translation.z = imu_translation[1];

  tf2::Quaternion rot;
  rot.setRotation({imu_rotation[0],
                   imu_rotation[1],
                   imu_rotation[2]}, imu_rotation[3]);

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

double InertialUnit::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
