#include "ros/ros.h"
#include "webots/Gyro.hpp"
#include "webots/Supervisor.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace AutomatED {

class Gyro {
public:
  Gyro(webots::Supervisor *webots_supervisor,
               ros::NodeHandle *ros_handle);
  ~Gyro();

  void publishGyro();

private:
  // Handlers
  webots::Supervisor *wb;
  ros::NodeHandle *nh;

  // Webots devices
  webots::Gyro *gyro;

  // ROS parameters
  std::string gyro_name;
  int sampling_period;
  std::string gyro_pub_topic;

  // ROS publisher
  ros::Publisher gyro_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();
};

}
