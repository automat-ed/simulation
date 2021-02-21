#include "sensors/gyro.hpp"
#include "webots/Gyro.hpp"
#include "sensor_msgs/Imu.h"
#include "webots/Supervisor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"



using namespace AutomatED;

Gyro::Gyro(webots::Supervisor *webots_supervisor,
                           ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("gyro/name", gyro_name, "gyro");
  nh->param("gyro/sampling_period", sampling_period, 32);
  nh->param<std::string>("gyro/data_topic", gyro_data_topic,
                         "gyro/data");

  // Create publishers
  gyro_data_pub = nh->advertise<sensor_msgs::Imu>(gyro_data_topic, 1);

  // Setup IMU device
  gyro = wb->getGyro(gyro_name);
  gyro->enable(sampling_period);

  // Publish tf
  publishTF();
}

Gyro::~Gyro() {
  gyro_data_pub.shutdown();
  gyro->disable();
}

void Gyro::publishGyroData() {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = gyro->getName();

  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 0.0;
  msg.orientation_covariance[0] = -1.0; // means no orientation information

  msg.angular_velocity.x = gyro->getValues()[0];
  msg.angular_velocity.y = gyro->getValues()[1];
  msg.angular_velocity.z = gyro->getValues()[2];
  for (int i = 0; i < 9; ++i)  // means "covariance unknown"
    msg.angular_velocity_covariance[i] = 0;

  msg.linear_acceleration.x = 0.0;
  msg.linear_acceleration.y = 0.0;
  msg.linear_acceleration.z = 0.0;
  msg.linear_acceleration_covariance[0] = -1.0; // means no linear_acceleration information

  gyro_data_pub.publish(msg);
}

void Gyro::publishTF() {
  // Get gyro node
  webots::Node *gyro_node = wb->getFromDevice(gyro);

  // Get gyro translation
  webots::Field *gyro_translation_field = gyro_node->getField("translation");
  const double *gyro_translation = gyro_translation_field->getSFVec3f();

  // Get gyro rotation
  webots::Field *gyro_rotation_field = gyro_node->getField("rotation");
  const double *gyro_rotation = gyro_rotation_field->getSFRotation();

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = gyro->getName();

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = gyro_translation[0];
  msg.transform.translation.y = -1 * gyro_translation[2];
  msg.transform.translation.z = gyro_translation[1];

  tf2::Quaternion rot;
  rot[0] = gyro_rotation[1];
  rot[1] = gyro_rotation[2];
  rot[2] = gyro_rotation[3];
  rot[3] = gyro_rotation[0];

  tf2::Quaternion webots_to_ros;
  webots_to_ros.setRPY(-1.5707, 0, 0);

  tf2::Quaternion quat = webots_to_ros * rot;
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}
