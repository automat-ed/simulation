#include "ros/ros.h"
#include "sensors/inertialUnit.hpp"
#include "webots/InertialUnit.hpp"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"


using namespace AutomatED;

InertialUnit::InertialUnit(webots::InertialUnit *device, ros::NodeHandle *ros_handle) {
  imu = device;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("imu/sampling_period", sampling_period, 32);
  nh->param<std::string>("imu/imu_quaternion_topic", imu_quaternion_topic,
                         "imu/imu_quaternion");

  // Create publishers
  imu_quaternion_pub = 
      nh->advertise<sensor_msgs::Imu>(imu_quaternion_topic, 1);

  // Enable IMU device
  imu->enable(sampling_period);
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
  for (int i = 0; i < 9; ++i)  // unknown covariance
    imu_msg.orientation_covariance[i] = 0;
  imu_msg.angular_velocity.x = 0.0;
  imu_msg.angular_velocity.y = 0.0;
  imu_msg.angular_velocity.z = 0.0;
  imu_msg.angular_velocity_covariance[0] = -1;  // no angular_velocity information
  imu_msg.linear_acceleration.x = 0.0;
  imu_msg.linear_acceleration.y = 0.0;
  imu_msg.linear_acceleration.z = 0.0;
  imu_msg.linear_acceleration_covariance[0] = -1.0;  // no linear_acceleration information

  imu_quaternion_pub.publish(imu_msg);
}