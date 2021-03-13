#include "sensors/lidar.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Lidar.hpp"
#include "webots/Supervisor.hpp"
#include <random>
#include <algorithm>

using namespace AutomatED;

Lidar::Lidar(webots::Supervisor *webots_supervisor,
             ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("lidar/name", lidar_name, "RobotisLds01");
  nh->param<std::string>("lidar/name", frame_id, "lidar");
  nh->param("lidar/sampling_period", sampling_period, 32);
  nh->param<std::string>("lidar/ground_truth_topic", ground_truth_topic,
                         "/lidar/ground_truth");
  nh->param<std::string>("lidar/noise_topic", noise_topic, "/lidar/data");
  nh->param("lidar/noise_mean", noise_mean, 0.0);
  nh->param("lidar/noise_std", noise_std, 0.01);
  nh->param<int>("lidar/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub =
      nh->advertise<sensor_msgs::LaserScan>(ground_truth_topic, 1);
  noise_pub = nh->advertise<sensor_msgs::LaserScan>(noise_topic, 1);

  // Setup LiDAR device
  lidar = wb->getLidar(lidar_name);
  lidar->enable(sampling_period);

  // Publish static transform
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};
}

Lidar::~Lidar()
{
  // Clean up
  ground_truth_pub.shutdown();
  noise_pub.shutdown();
  lidar->disable();
}

void Lidar::publishLaserScan()
{
  for (int layer = 0; layer < lidar->getNumberOfLayers(); layer++)
  {
    const float *rangeImageVector = lidar->getLayerRangeImage(layer);

    // Publish ground truth
    sensor_msgs::LaserScan gt;
    gt.header.stamp = ros::Time::now();
    gt.header.frame_id = frame_id;

    gt.angle_min = -lidar->getFov() / 2.0;
    gt.angle_max = lidar->getFov() / 2.0;
    gt.angle_increment = lidar->getFov() / lidar->getHorizontalResolution();

    gt.scan_time = (double)lidar->getSamplingPeriod() / 1000.0;
    gt.time_increment = (double)lidar->getSamplingPeriod() /
                        (1000.0 * lidar->getHorizontalResolution());

    gt.range_min = lidar->getMinRange();
    gt.range_max = lidar->getMaxRange();
    for (int i = 0; i < lidar->getHorizontalResolution(); i++)
    {
      gt.ranges.push_back(rangeImageVector[i]);
    }

    ground_truth_pub.publish(gt);

    // Publish noisy data
    sensor_msgs::LaserScan msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;

    msg.angle_min = -lidar->getFov() / 2.0;
    msg.angle_max = lidar->getFov() / 2.0;
    msg.angle_increment = lidar->getFov() / lidar->getHorizontalResolution();

    msg.scan_time = (double)lidar->getSamplingPeriod() / 1000.0;
    msg.time_increment = (double)lidar->getSamplingPeriod() /
                         (1000.0 * lidar->getHorizontalResolution());

    msg.range_min = lidar->getMinRange();
    msg.range_max = lidar->getMaxRange();
    for (int i = 0; i < lidar->getHorizontalResolution(); i++)
    {
      if (rangeImageVector[i] == INFINITY)
      {
        msg.ranges.push_back(rangeImageVector[i]);
      }
      else
      {
        double noisy_range = rangeImageVector[i] + gaussianNoise();
        // Make sure value is between `range_min` and `range_max`
        noisy_range = std::max((double)msg.range_min, std::min(noisy_range, (double)msg.range_max));
        msg.ranges.push_back(noisy_range);
      }
    }

    noise_pub.publish(msg);
  }
}

void Lidar::publishTF()
{
  // Get lidar node
  webots::Node *lidar_node = wb->getFromDevice(lidar);

  // Get lidar translation
  webots::Field *lidar_translation_field = lidar_node->getField("translation");
  const double *lidar_translation = lidar_translation_field->getSFVec3f();

  ROS_DEBUG("Lidar translation: [%f, %f, %f]",
           lidar_translation[0],
           lidar_translation[1],
           lidar_translation[2]);

  // Get lidar rotation
  webots::Field *lidar_rotation_field = lidar_node->getField("rotation");
  const double *lidar_rotation = lidar_rotation_field->getSFRotation();

  ROS_DEBUG("Lidar rotation: [%f, %f, %f, %f]",
           lidar_rotation[0],
           lidar_rotation[1],
           lidar_rotation[2],
           lidar_rotation[3]);

  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = frame_id;

  // Translate from Webots to ROS coordinates
  msg.transform.translation.x = lidar_translation[0];
  msg.transform.translation.y = -1 * lidar_translation[2];
  msg.transform.translation.z = lidar_translation[1];

  tf2::Quaternion rot;
  rot.setRotation({lidar_rotation[0],
                   lidar_rotation[1],
                   lidar_rotation[2]}, lidar_rotation[3]);

  tf2::Quaternion ros_to_webots;
  ros_to_webots.setRPY(1.5708, 0, 0);

  tf2::Quaternion quat = ros_to_webots * rot;
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(msg);
}

double Lidar::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
