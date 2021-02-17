#include "ros/ros.h"
#include "webots/Lidar.hpp"

namespace AutomatED {

class Lidar {
public:
  Lidar(webots::Lidar *device, ros::NodeHandle *ros_handle);
  ~Lidar();

  void publishLaserScan();

private:
  webots::Lidar *lidar;
  ros::NodeHandle *nh;

  // ROS parameters
  int sampling_period;
  std::string laser_scan_topic;

  ros::Publisher laser_scan_pub;
};

} // namespace AutomatED