#include "ros/ros.h"
#include "webots/Lidar.hpp"

namespace AutomatED {

class Lidar {
public:
  Lidar(webots::Lidar *device, ros::NodeHandle *ros_handle);
  ~Lidar();

  void publishPointCloud();

private:
  webots::Lidar *lidar;
  ros::NodeHandle *nh;

  // ROS parameters
  int sampling_period;
  std::string point_cloud_topic;

  ros::Publisher point_cloud_pub;
};

} // namespace AutomatED