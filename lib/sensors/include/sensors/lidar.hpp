#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Lidar.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED {

class Lidar {
public:
  Lidar(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle);
  ~Lidar();

  void publishLaserScan();

private:
  // Handlers
  webots::Supervisor *wb;
  ros::NodeHandle *nh;

  // Webots devices
  webots::Lidar *lidar;

  // ROS parameters
  int sampling_period;
  std::string ground_truth_topic;
  std::string noise_topic;
  double noise_error;
  int noise_seed;
  std::string lidar_name;

  // ROS publisher
  ros::Publisher ground_truth_pub;
  ros::Publisher noise_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();

  // Noise
  std::mt19937 *gen;
  double gaussianNoise(double value);
};

} // namespace AutomatED
