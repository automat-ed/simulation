#include "ros/ros.h"
#include "webots/InertialUnit.hpp"

namespace AutomatED {

class InertialUnit {
public:
  InertialUnit(webots::InertialUnit *device, ros::NodeHandle *ros_handle);
  ~InertialUnit();

  void publishImuQuaternion();

private:
  webots::InertialUnit *imu;
  ros::NodeHandle *nh;

  // ROS parameters
  int sampling_period;
  std::string imu_quaternion_topic;

  ros::Publisher imu_quaternion_pub;
};


} 
