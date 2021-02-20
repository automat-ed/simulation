#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "webots/Motor.hpp"

namespace AutomatED {
    
class DiffSteering {
public:
  DiffSteering(webots::Motor *motors[], ros::NodeHandle *ros_handle);
  ~DiffSteering();
  void keyboardInput();

private:
  webots::Motor **wheels;
  ros::NodeHandle *nh;
  double wheel_separation;
  double wheel_radius;
  
  // Create Subscriber
  ros::Subscriber cmd_vel_sub;

  void velocityCallback(const geometry_msgs::TwistConstPtr& cmd);
  void turnOnMotors();
  void shutDownMotors();
};

} // namespace AutomatED
