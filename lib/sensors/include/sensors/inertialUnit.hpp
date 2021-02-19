#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/InertialUnit.hpp"
#include "webots/Supervisor.hpp"

namespace AutomatED {

class InertialUnit {
public:
  InertialUnit(webots::Supervisor *webots_supervisor,
               ros::NodeHandle *ros_handle);
  ~InertialUnit();

  void publishImuQuaternion();

private:
  // Handlers
  webots::Supervisor *wb;
  ros::NodeHandle *nh;

  // Webots devices
  webots::InertialUnit *imu;

  // ROS parameters
  std::string imu_name;
  int sampling_period;
  std::string imu_quaternion_topic;

  // ROS publisher
  ros::Publisher imu_quaternion_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();
};

} // namespace AutomatED
