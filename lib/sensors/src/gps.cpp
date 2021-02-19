#include "webots/GPS.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensors/gps.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "webots/Supervisor.hpp"
#include "webots_ros/Float64Stamped.h"

using namespace AutomatED;

GPS::GPS(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("gps/name", gps_name, "gps");
  nh->param("gps/sampling_period", sampling_period, 32);
  nh->param<std::string>("gps/coordinate_topic", gps_coordinate_topic,
                         "gps/coordinates");
  nh->param<std::string>("gps/speed_topic", gps_speed_topic, "gps/speed");

  // Create publishers
  gps_coordinate_pub =
      nh->advertise<sensor_msgs::NavSatFix>(gps_coordinate_topic, 1);
  gps_speed_pub = nh->advertise<webots_ros::Float64Stamped>(gps_speed_topic, 1);

  // Setup GPS device
  gps = wb->getGPS(gps_name);
  gps->enable(sampling_period);

  // Publish static transform
  publishTF();
}

GPS::~GPS() {
  gps_coordinate_pub.shutdown();
  gps_speed_pub.shutdown();
  gps->disable();
}

void GPS::publishGPSCoordinate() {
  sensor_msgs::NavSatFix gps_value_msg;
  gps_value_msg.header.stamp = ros::Time::now();
  gps_value_msg.header.frame_id = gps->getName();
  gps_value_msg.latitude = gps->getValues()[0];
  gps_value_msg.longitude = gps->getValues()[2];
  gps_value_msg.altitude = gps->getValues()[1];
  gps_value_msg.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gps_value_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_coordinate_pub.publish(gps_value_msg);
}

void GPS::publishGPSSpeed() {
  webots_ros::Float64Stamped speed_value_msg;
  speed_value_msg.header.stamp = ros::Time::now();
  speed_value_msg.data = gps->getSpeed();
  gps_speed_pub.publish(speed_value_msg);
}

void GPS::publishTF() {
  // Get gps node
  webots::Node *gps_node = wb->getFromDevice(gps);

  // Get gps translation
  webots::Field *gps_translation_field = gps_node->getField("translation");
  const double *gps_translation = gps_translation_field->getSFVec3f();

  // Get gps rotation
  webots::Field *gps_rotation_field = gps_node->getField("rotation");
  const double *gps_rotation = gps_rotation_field->getSFRotation();

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = gps->getName();

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = gps_translation[0];
  msg.transform.translation.y = -1 * gps_translation[2];
  msg.transform.translation.z = gps_translation[1];

  tf2::Quaternion rot;
  rot[0] = gps_rotation[1];
  rot[1] = gps_rotation[2];
  rot[2] = gps_rotation[3];
  rot[3] = gps_rotation[0];

  tf2::Quaternion webots_to_ros;
  webots_to_ros.setRPY(-1.5707, 0, 0);

  tf2::Quaternion quat = webots_to_ros * rot;
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}