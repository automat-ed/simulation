#include "sensors/lidar.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Lidar.hpp"
#include "webots/Supervisor.hpp"

using namespace AutomatED;

Lidar::Lidar(webots::Supervisor *webots_supervisor,
             ros::NodeHandle *ros_handle) {
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("lidar/name", lidar_name, "RobotisLds01");
  nh->param("lidar/sampling_period", sampling_period, 32);
  nh->param<std::string>("lidar/laser_scan_topic", laser_scan_topic,
                         "lidar/laser_scan");

  // Create publishers
  laser_scan_pub = nh->advertise<sensor_msgs::LaserScan>(laser_scan_topic, 1);

  // Setup LiDAR device
  lidar = wb->getLidar(lidar_name);
  lidar->enable(sampling_period);

  // Publish static transform
  publishTF();
}

Lidar::~Lidar() {
  // Clean up
  laser_scan_pub.shutdown();
  lidar->disable();
}

void Lidar::publishLaserScan() {
  for (int layer = 0; layer < lidar->getNumberOfLayers(); ++layer) {
    const float *rangeImageVector = lidar->getLayerRangeImage(layer);

    sensor_msgs::LaserScan laser_scan_msg;
    laser_scan_msg.header.stamp = ros::Time::now();
    laser_scan_msg.header.frame_id = lidar->getName();

    laser_scan_msg.angle_min = -lidar->getFov() / 2.0;
    laser_scan_msg.angle_max = lidar->getFov() / 2.0;
    laser_scan_msg.angle_increment =
        lidar->getFov() / lidar->getHorizontalResolution();

    laser_scan_msg.scan_time = (double)lidar->getSamplingPeriod() / 1000.0;
    laser_scan_msg.time_increment = (double)lidar->getSamplingPeriod() /
                                    (1000.0 * lidar->getHorizontalResolution());

    laser_scan_msg.range_min = lidar->getMinRange();
    laser_scan_msg.range_max = lidar->getMaxRange();
    for (int i = 0; i < lidar->getHorizontalResolution(); ++i) {
      laser_scan_msg.ranges.push_back(rangeImageVector[i]);
    }

    laser_scan_pub.publish(laser_scan_msg);
  }
}

void Lidar::publishTF() {
  // Get lidar node
  webots::Node *lidar_node = wb->getFromDevice(lidar);

  // Get lidar translation
  webots::Field *lidar_translation_field = lidar_node->getField("translation");
  const double *lidar_translation = lidar_translation_field->getSFVec3f();

  // Get lidar rotation
  webots::Field *lidar_rotation_field = lidar_node->getField("rotation");
  const double *lidar_rotation = lidar_rotation_field->getSFRotation();

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = lidar->getName();

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = lidar_translation[0];
  msg.transform.translation.y = -1 * lidar_translation[2];
  msg.transform.translation.z = lidar_translation[1];

  tf2::Quaternion rot;
  rot[0] = lidar_rotation[1];
  rot[1] = lidar_rotation[2];
  rot[2] = lidar_rotation[3];
  rot[3] = lidar_rotation[0];

  tf2::Quaternion webots_to_ros;
  webots_to_ros.setRPY(-1.5707, 0, 0);

  tf2::Quaternion quat = webots_to_ros * rot;
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}