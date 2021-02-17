#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensors/lidar.hpp"
#include "webots/Lidar.hpp"

using namespace AutomatED;

Lidar::Lidar(webots::Lidar *device, ros::NodeHandle *ros_handle) {
  lidar = device;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("lidar/sampling_period", sampling_period, 32);
  nh->param<std::string>("lidar/laser_scan_topic", laser_scan_topic,
                         "lidar/laser_scan");
  // Create publishers
  laser_scan_pub =
      nh->advertise<sensor_msgs::LaserScan>(laser_scan_topic, 1);

  // Enable lidar device
  lidar->enable(sampling_period);
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
    laser_scan_msg.angle_increment = lidar->getFov() / lidar->getHorizontalResolution();
    laser_scan_msg.time_increment = (double)lidar->getSamplingPeriod() / (1000.0 * lidar->getHorizontalResolution());
    laser_scan_msg.scan_time = (double)lidar->getSamplingPeriod() / 1000.0;
    laser_scan_msg.range_min = lidar->getMinRange();
    laser_scan_msg.range_max = lidar->getMaxRange();
    for (int i = 0; i < lidar->getHorizontalResolution(); ++i)
      laser_scan_msg.ranges.push_back(rangeImageVector[i]);
    laser_scan_pub.publish(laser_scan_msg);
  }
}
