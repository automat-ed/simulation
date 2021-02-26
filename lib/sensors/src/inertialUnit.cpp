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
                           ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("imu/name", imu_name, "imu");
  nh->param("imu/sampling_period", sampling_period, 32);
  nh->param<std::string>("imu/ground_truth_topic", ground_truth_topic,
                         "/imu/ground_truth");
  nh->param<std::string>("imu/noise_topic", noise_topic, "/imu/data");
  nh->param("accelerometer/noise_error", noise_error, 0.05);
  nh->param<int>("accelerometer/noise_seed", noise_seed, 17);

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
}

InertialUnit::~InertialUnit() {
  ground_truth_pub.shutdown();
  noise_pub.shutdown();
  imu->disable();
}

void InertialUnit::publishImuQuaternion() {
  // Get sensor reading
  const double *reading = imu->getQuaternion();

  // Calculate quaternion
  tf2::Quaternion orientation(reading[0], reading[1], reading[2], reading[3]);
  tf2::Quaternion orientationRosFix(0.5, 0.5, 0.5, 0.5);
  orientation = orientation * orientationRosFix;

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = imu->getName();
  gt.orientation.x = orientation.getX();
  gt.orientation.y = orientation.getY();
  gt.orientation.z = orientation.getZ();
  gt.orientation.w = orientation.getW();
  gt.angular_velocity_covariance[0] = -1;      // no information
  gt.linear_acceleration_covariance[0] = -1.0; // no information
  ground_truth_pub.publish(gt);

  // Publish noisy data
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = imu->getName();
  msg.orientation.x = orientation.getX() + gaussianNoise(orientation.getX());
  msg.orientation.y = orientation.getY() + gaussianNoise(orientation.getY());
  msg.orientation.z = orientation.getZ() + gaussianNoise(orientation.getZ());
  msg.orientation.w = orientation.getW() + gaussianNoise(orientation.getW());
  msg.angular_velocity_covariance[0] = -1;      // no information
  msg.linear_acceleration_covariance[0] = -1.0; // no information
  noise_pub.publish(msg);
}

void InertialUnit::publishTF() {
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
  msg.child_frame_id = imu->getName();

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = imu_translation[0];
  msg.transform.translation.y = -1 * imu_translation[2];
  msg.transform.translation.z = imu_translation[1];

  tf2::Quaternion rot;
  rot[0] = imu_rotation[1];
  rot[1] = imu_rotation[2];
  rot[2] = imu_rotation[3];
  rot[3] = imu_rotation[0];

  tf2::Quaternion webots_to_ros;
  webots_to_ros.setRPY(-1.5707, 0, 0);

  tf2::Quaternion quat = webots_to_ros * rot;
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}

double InertialUnit::gaussianNoise(double value) {
  std::normal_distribution<double> d{0, noise_error * value};
  return d(*gen);
}
