#include "sensors/inertialUnit.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/InertialUnit.hpp"
#include "webots/Supervisor.hpp"

using namespace AutomatED;

InertialUnit::InertialUnit(webots::Supervisor *webots_supervisor,
                           ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("imu/name", imu_name, "imu");
  nh->param("imu/sampling_period", sampling_period, 32);
  nh->param<std::string>("imu/quaternion_topic", imu_quaternion_topic,
                         "imu/quaternion");

  // Create publishers
  imu_quaternion_pub = nh->advertise<sensor_msgs::Imu>(imu_quaternion_topic, 1);

  // Setup IMU device
  imu = wb->getInertialUnit(imu_name);
  imu->enable(sampling_period);

  // Publish tf
  publishTF();
}

InertialUnit::~InertialUnit() {
  imu_quaternion_pub.shutdown();
  imu->disable();
}

void InertialUnit::publishImuQuaternion() {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = imu->getName();

  tf2::Quaternion orientation(imu->getQuaternion()[0], imu->getQuaternion()[1],
                              imu->getQuaternion()[2], imu->getQuaternion()[3]);
  tf2::Quaternion orientationRosFix(0.5, 0.5, 0.5, 0.5);
  orientation = orientation * orientationRosFix;

  imu_msg.orientation.x = orientation.getX();
  imu_msg.orientation.y = orientation.getY();
  imu_msg.orientation.z = orientation.getZ();
  imu_msg.orientation.w = orientation.getW();
  for (int i = 0; i < 9; ++i) // unknown covariance
    imu_msg.orientation_covariance[i] = 0;
  imu_msg.angular_velocity.x = 0.0;
  imu_msg.angular_velocity.y = 0.0;
  imu_msg.angular_velocity.z = 0.0;
  imu_msg.angular_velocity_covariance[0] =
      -1; // no angular_velocity information
  imu_msg.linear_acceleration.x = 0.0;
  imu_msg.linear_acceleration.y = 0.0;
  imu_msg.linear_acceleration.z = 0.0;
  imu_msg.linear_acceleration_covariance[0] =
      -1.0; // no linear_acceleration information

  imu_quaternion_pub.publish(imu_msg);
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
