#include "ros/ros.h"
#include "webots/Motor.hpp"
#include "webots/Supervisor.hpp"


namespace AutomatED {

class WheelOdometry {
public:
  WheelOdometry(webots::Supervisor *webots_supervisor,
                ros::NodeHandle *ros_handle);
  ~WheelOdometry();

  void publishWheelOdometry();

private:
  // Handlers
  webots::Supervisor *wb;
  ros::NodeHandle *nh;

  // Webots devices
  webots::Motor *wheels[4];

  // ROS parameters
  int sampling_period;
  std::string wheel_odometry_twist_topic;
  double wheel_separation;
  double wheel_radius;

  // ROS publisher
  ros::Publisher wheel_odometry_pub;
};

}
