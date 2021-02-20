#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
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
  webots::Motor *wheels[];

  // ROS parameters
  std::string wheel_names[];
  int sampling_period;
  std::string wheel_odometry_topic;

  // ROS publisher
  ros::Publisher wheel_odometry_pub;
};

}
