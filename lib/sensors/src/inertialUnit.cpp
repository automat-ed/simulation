#include "sensors/inertialUnit.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/InertialUnit.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

InertialUnit::InertialUnit(webots::Supervisor *webots_supervisor,
                           ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("imu/name", imu_name, "imu");
  nh->param<std::string>("imu/frame_id", frame_id, "imu");
  nh->param("imu/sampling_period", sampling_period, 32);
  nh->param<std::string>("imu/ground_truth_topic", ground_truth_topic,
                         "/imu/ground_truth");
  nh->param<std::string>("imu/noise_topic", noise_topic, "/imu/data");
  nh->param("imu/noise_mean", noise_mean, 0.0);
  nh->param("imu/noise_std", noise_std, 0.02);
  nh->param("imu/bias_mean", bias_mean, 0.005);
  nh->param("imu/bias_std", bias_std, 0.001);
  nh->param<int>("imu/noise_seed", noise_seed, 17);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Imu>(ground_truth_topic, 1);
  noise_pub = nh->advertise<sensor_msgs::Imu>(noise_topic, 1);

  // Setup IMU device
  imu = wb->getInertialUnit(imu_name);
  imu->enable(sampling_period);

  // Publish tf
  publishTF();

  // Initialize generator with seed
  gen = new std::mt19937{(long unsigned int)noise_seed};

  // Sample bias
  std::normal_distribution<double> d{bias_mean, bias_std};
  bias = d(*gen);
}

InertialUnit::~InertialUnit()
{
  ground_truth_pub.shutdown();
  noise_pub.shutdown();
  imu->disable();
}

void InertialUnit::publishImuQuaternion()
{
  // Get sensor reading
  const double *reading = imu->getRollPitchYaw();

  // Extract data
  double webots_yaw = reading[2];

  // Shift reading to get 0 when facing East
  webots_yaw += 1.5708;

  // Convert to Quaternion
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, webots_yaw);
  quaternion.normalize();

  // Publish ground truth
  sensor_msgs::Imu gt;
  gt.header.stamp = ros::Time::now();
  gt.header.frame_id = frame_id;
  gt.orientation.x = quaternion.getX();
  gt.orientation.y = quaternion.getY();
  gt.orientation.z = quaternion.getZ();
  gt.orientation.w = quaternion.getW();
  gt.angular_velocity_covariance[0] = -1;      // no information
  gt.linear_acceleration_covariance[0] = -1.0; // no information
  ground_truth_pub.publish(gt);
}

void InertialUnit::publishTF()
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

double InertialUnit::gaussianNoise()
{
  std::normal_distribution<double> d{noise_mean, noise_std};
  return d(*gen);
}
