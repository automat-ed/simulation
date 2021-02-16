#include "webots/GPS.hpp"
#include "sensors/gps.hpp"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "webots_ros/Float64Stamped.h"
#include "ros/ros.h"

using namespace AutomatED;

GPS::GPS(webots::GPS *device, ros::NodeHandle *ros_handle) {
  gps = device;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("gps/sampling_period", sampling_period, 32);
  nh->param<std::string>("gps/gps_coordinate_topic", gps_coordinate_topic,
                          "/gps/gps_coordinate");
  nh->param<std::string>("gps/gps_speed_topic", gps_speed_topic,
                          "/gps/speed");

  // Create publishers
  gps_coordinate_pub = 
      nh->advertise<sensor_msgs::NavSatFix>(gps_coordinate_topic, 10);
  gps_speed_pub = 
      nh->advertise<webots_ros::Float64Stamped>(gps_speed_topic, 10);

  // Enable GPS device
  gps->enable(sampling_period);
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
  gps_value_msg.longitude = gps->getValues()[1];
  gps_value_msg.altitude = gps->getValues()[2];
  gps_value_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gps_value_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_coordinate_pub.publish(gps_value_msg);
}

void GPS::publishGPSSpeed() {
  webots_ros::Float64Stamped speed_value_msg;
  speed_value_msg.header.stamp = ros::Time::now();
  speed_value_msg.data = gps->getSpeed();
  gps_speed_pub.publish(speed_value_msg);
}
