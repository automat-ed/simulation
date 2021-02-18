#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "webots/Motor.hpp"

namespace AutomatED {
    
class DiffSteering {
public:
  DiffSteering(webots::Motor motors[4], ros::NodeHandle *ros_handle);
  ~DiffSteering();

private:
  webots::Motor wheels[4];
  ros::NodeHandle *nh;
  const double wheel_separation;
  const double wheel_radius;
  
  // Create Subscriber
  ros::Subscriber cmd_vel_sub;

  void subscribeVelocity(const geometry_msgs::Twist& cmd);
  void turnOnMotors();
  void shutDownMotors();
  void keyboardInput();
}

} // namespace AutomatED
