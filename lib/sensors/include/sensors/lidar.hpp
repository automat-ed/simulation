#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Lidar.hpp"
#include "webots/Supervisor.hpp"

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
  std::string laser_scan_topic;
  std::string lidar_name;

  // ROS publisher
  ros::Publisher laser_scan_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();
};

} // namespace AutomatED
