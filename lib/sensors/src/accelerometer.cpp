#include "sensors/accelerometer.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Accelerometer.hpp"
#include "webots/Supervisor.hpp"

using namespace AutomatED;

Accelerometer::Accelerometer(webots::Supervisor *webots_supervisor,
                           ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("accelerometer/name", accelerometer_name, "accelerometer");
  nh->param("accelerometer/sampling_period", sampling_period, 32);
  nh->param<std::string>("accelerometer/data_topic", accelerometer_data_topic,
                         "accelerometer/data");

  // Create publishers
  accelerometer_data_pub = nh->advertise<sensor_msgs::Imu>(accelerometer_data_topic, 1);

  // Setup accelerometer device
  accelerometer = wb->getAccelerometer(accelerometer_name);
  accelerometer->enable(sampling_period);

  // Publish tf
  publishTF();
}

Accelerometer::~Accelerometer() {
  accelerometer_data_pub.shutdown();
  accelerometer->disable();
}

void Accelerometer::publishAccelerometer() {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = accelerometer->getName();

  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 0.0;
  msg.orientation_covariance[0] = -1.0;  // means no orientation information

  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = 0.0;
  msg.angular_velocity_covariance[0] = -1; // no angular_velocity information

  msg.linear_acceleration.x = accelerometer->getValues()[0];
  msg.linear_acceleration.y = accelerometer->getValues()[1];
  msg.linear_acceleration.z = accelerometer->getValues()[2];
  for (int i = 0; i < 9; ++i)  // means "covariance unknown"
    msg.linear_acceleration_covariance[i] = 0;

  accelerometer_data_pub.publish(msg);
}

void Accelerometer::publishTF() {
  // Get accelerometer node
  webots::Node *accelerometer_node = wb->getFromDevice(accelerometer);

  // Get accelerometer translation
  webots::Field *accelerometer_translation_field = accelerometer_node->getField("translation");
  const double *accelerometer_translation = accelerometer_translation_field->getSFVec3f();

  // Get accelerometer rotation
  webots::Field *accelerometer_rotation_field = accelerometer_node->getField("rotation");
  const double *accelerometer_rotation = accelerometer_rotation_field->getSFRotation();

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = accelerometer->getName();

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = accelerometer_translation[0];
  msg.transform.translation.y = -1 * accelerometer_translation[2];
  msg.transform.translation.z = accelerometer_translation[1];

  tf2::Quaternion rot;
  rot[0] = accelerometer_rotation[1];
  rot[1] = accelerometer_rotation[2];
  rot[2] = accelerometer_rotation[3];
  rot[3] = accelerometer_rotation[0];

  tf2::Quaternion webots_to_ros;
  webots_to_ros.setRPY(-1.5707, 0, 0);

  tf2::Quaternion quat = webots_to_ros * rot;
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}
