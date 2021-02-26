#include "webots/GPS.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensors/gps.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "webots/Supervisor.hpp"
#include "webots_ros/Float64Stamped.h"
#include <random>

using namespace AutomatED;

GPS::GPS(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("gps/name", gps_name, "gps");
  nh->param("gps/sampling_period", sampling_period, 32);
  nh->param<std::string>("gps/gt_coordinate_topic", gt_coordinate_topic,
                         "/gps/ground_truth/coordinates");
  nh->param<std::string>("gps/coordinate_topic", coordinate_topic,
                         "/gps/coordinates");
  nh->param<std::string>("gps/gt_speed_topic", gt_speed_topic,
                         "/gps/ground_truth/speed");
  nh->param<std::string>("gps/speed_topic", speed_topic, "/gps/speed");
  nh->param("gps/noise_error", noise_error, 0.05);
  nh->param("gps/noise_seed", noise_seed, 17);

  // Create publishers
  gt_coordinate_pub =
      nh->advertise<sensor_msgs::NavSatFix>(gt_coordinate_topic, 1);
  coordinate_pub = nh->advertise<sensor_msgs::NavSatFix>(coordinate_topic, 1);
  gt_speed_pub = nh->advertise<webots_ros::Float64Stamped>(gt_speed_topic, 1);
  speed_pub = nh->advertise<webots_ros::Float64Stamped>(speed_topic, 1);

  // Setup GPS device
  gps = wb->getGPS(gps_name);
  gps->enable(sampling_period);

  // Publish static transform
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};
}

GPS::~GPS() {
  gt_coordinate_pub.shutdown();
  coordinate_pub.shutdown();
  gt_speed_pub.shutdown();
  speed_pub.shutdown();
  gps->disable();
}

void GPS::publishGPSCoordinate() {
  // Get Coordinate reading from GPS
  const double *reading = gps->getValues();

  // Publish ground truth
  sensor_msgs::NavSatFix gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = gps->getName();
  gt.latitude = reading[0];
  gt.longitude = reading[2];
  gt.altitude = reading[1];
  gt.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gt.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gt_coordinate_pub.publish(gt);

  // Publish data with noise
  sensor_msgs::NavSatFix msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = gps->getName();
  msg.latitude = reading[0] + gaussianNoise(reading[0]);
  msg.longitude = reading[2] + gaussianNoise(reading[2]);
  msg.altitude = reading[1] + gaussianNoise(reading[1]);
  msg.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  coordinate_pub.publish(msg);
}

void GPS::publishGPSSpeed() {
  // Get speed data from GPS
  const double reading = gps->getSpeed();

  // Publish ground truth
  webots_ros::Float64Stamped gt;
  gt.header.stamp = ros::Time::now();
  gt.data = reading;
  gt_speed_pub.publish(gt);

  // Publish data with noise
  webots_ros::Float64Stamped msg;
  msg.header.stamp = ros::Time::now();
  msg.data = reading + gaussianNoise(reading);
  speed_pub.publish(msg);
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

double GPS::gaussianNoise(double value) {
  std::normal_distribution<double> d{0, noise_error * value};
  return d(*gen);
}
