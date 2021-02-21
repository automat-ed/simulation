#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Accelerometer.hpp"
#include "webots/Supervisor.hpp"

namespace AutomatED {

class Accelerometer {
public:
  Accelerometer(webots::Supervisor *webots_supervisor,
               ros::NodeHandle *ros_handle);
  ~Accelerometer();

  void publishAccelerometer();

private:
  // Handlers
  webots::Supervisor *wb;
  ros::NodeHandle *nh;

  // Webots devices
  webots::Accelerometer *accelerometer;

  // ROS parameters
  std::string accelerometer_name;
  int sampling_period;
  std::string accelerometer_data_topic;

  // ROS publisher
  ros::Publisher accelerometer_data_pub;

  // Tf2
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  void publishTF();
};

} // namespace AutomatED
