#include "webots/Lidar.hpp"
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensors/lidar.hpp"

using namespace AutomatED;

Lidar::Lidar(webots::Lidar *device, ros::NodeHandle *ros_handle) {
  lidar = device;
  nh = ros_handle;

  // Get ROS parameters
  nh->param("lidar/sampling_period", sampling_period, 32);
  nh->param<std::string>("lidar/point_cloud_topic", point_cloud_topic,
                         "/lidar/point_cloud");
  // Create publishers
  point_cloud_pub =
      nh->advertise<sensor_msgs::PointCloud>(point_cloud_topic, 10);

  // Enable lidar device
  lidar->enable(sampling_period);
  lidar->enablePointCloud();
}

Lidar::~Lidar() {
  // Clean up
  point_cloud_pub.shutdown();
  lidar->disablePointCloud();
  lidar->disable();
}

void Lidar::publishPointCloud() {
  // Get point cloud from Webots
  const webots::LidarPoint *wb_point_cloud = lidar->getPointCloud();

  // Create point cloud message
  sensor_msgs::PointCloud point_cloud_msg;
  point_cloud_msg.header.stamp = ros::Time::now();
  point_cloud_msg.header.frame_id = lidar->getName();

  sensor_msgs::ChannelFloat32 layerChannel;
  for (int i = 0; i < lidar->getNumberOfPoints(); i++) {
    geometry_msgs::Point32 point;
    point.x = wb_point_cloud[i].x;
    point.y = wb_point_cloud[i].y;
    point.z = wb_point_cloud[i].z;
    point_cloud_msg.points.push_back(point);

    layerChannel.values.push_back(wb_point_cloud[i].layer_id);
  }
  point_cloud_msg.channels.push_back(layerChannel);

  // Publish
  point_cloud_pub.publish(point_cloud_msg);
}