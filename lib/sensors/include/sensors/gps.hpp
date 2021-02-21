#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/GPS.hpp"
#include "webots/Supervisor.hpp"

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
  std::string gps_coordinate_topic;
  std::string gps_speed_topic;

  // ROS publishers
  ros::Publisher gps_coordinate_pub;
  ros::Publisher gps_speed_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();
};

} // namespace AutomatED
