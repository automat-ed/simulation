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

GPS::GPS(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("gps/name", gps_name, "gps");
  nh->param<std::string>("gps/frame_id", frame_id, "gps");
  nh->param("gps/sampling_period", sampling_period, 32);
  nh->param<std::string>("gps/gt_coordinate_topic", gt_coordinate_topic,
                         "/gps/ground_truth/coordinates");
  nh->param<std::string>("gps/coordinate_topic", coordinate_topic,
                         "/gps/coordinates");
  nh->param("gps/noise_mean", noise_mean, 0.0);
  nh->param("gps/noise_std", noise_std, 0.1);
  nh->param("gps/bias_mean", bias_mean, 0.1);
  nh->param("gps/bias_std", bias_std, 0.001);
  nh->param("gps/noise_seed", noise_seed, 17);

  // Create publishers
  gt_coordinate_pub =
      nh->advertise<sensor_msgs::NavSatFix>(gt_coordinate_topic, 1);
  coordinate_pub = nh->advertise<sensor_msgs::NavSatFix>(coordinate_topic, 1);

  // Setup GPS device
  gps = wb->getGPS(gps_name);
  gps->enable(sampling_period);

  // Publish static transform
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> d{bias_mean, bias_std};
  bias = d(*gen);
}

GPS::~GPS()
{
  gt_coordinate_pub.shutdown();
  coordinate_pub.shutdown();
  gps->disable();
}

void GPS::publishGPSCoordinate()
{
  // Get Coordinate reading from GPS
  const double *reading = gps->getValues();

  // Publish ground truth
  sensor_msgs::NavSatFix gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;
  gt.latitude = reading[0];
  gt.longitude = reading[1];
  gt.altitude = reading[2];
  gt.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  gt.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gt_coordinate_pub.publish(gt);
}

void GPS::publishTF()
{
  // Create transform msg
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.child_frame_id = frame_id;

  msg.transform.translation.x = 0;
  msg.transform.translation.y = 0;
  msg.transform.translation.z = 0;

  msg.transform.rotation.x = 0;
  msg.transform.rotation.y = 0;
  msg.transform.rotation.z = 0;
  msg.transform.rotation.w = 1;

  static_broadcaster.sendTransform(msg);
}

double GPS::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
