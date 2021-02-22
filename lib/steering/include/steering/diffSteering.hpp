#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "webots/Motor.hpp"

namespace AutomatED {

class DiffSteering {
public:
  DiffSteering(std::vector<webots::Motor *> motors,
               ros::NodeHandle *ros_handle);
  ~DiffSteering();

private:
  std::vector<webots::Motor *> wheels;
  ros::NodeHandle *nh;
  double wheel_separation;
  double wheel_radius;
  int wheel_count;

  // Create Subscriber
  ros::Subscriber cmd_vel_sub;

  void velocityCallback(const geometry_msgs::TwistConstPtr &cmd);
  void stopMotors();
};

} // namespace AutomatED
