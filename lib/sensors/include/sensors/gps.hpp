#include "ros/ros.h"
#include "webots/GPS.hpp"

namespace AutomatED {

class GPS {
public:
  GPS(webots::GPS *device, ros::NodeHandle *ros_handle);
  ~GPS();

  void publishGPSCoordinate();
  void publishGPSSpeed();

private:
  webots::GPS *gps;
  ros::NodeHandle *nh;

  // ROS parameters
  int sampling_period;
  std::string gps_coordinate_topic;
  std::string gps_speed_topic;

  ros::Publisher gps_coordinate_pub;
  ros::Publisher gps_speed_pub;

};

} // namespace AutomatED