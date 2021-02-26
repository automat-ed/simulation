#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/GPS.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED {

class GPS {
public:
  GPS(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle);
  ~GPS();

  void publishGPSCoordinate();
  void publishGPSSpeed();

private:
  webots::Supervisor *wb;
  ros::NodeHandle *nh;

  // Webots Devices
  webots::GPS *gps;

  // ROS parameters
  std::string gps_name;
  int sampling_period;
  std::string gt_coordinate_topic;
  std::string gt_speed_topic;
  std::string coordinate_topic;
  std::string speed_topic;
  double noise_error;
  int noise_seed;

  // ROS publishers
  ros::Publisher gt_coordinate_pub;
  ros::Publisher coordinate_pub;
  ros::Publisher gt_speed_pub;
  ros::Publisher speed_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();

  // Noise
  std::mt19937 *gen;
  double gaussianNoise(double value);
};

} // namespace AutomatED
