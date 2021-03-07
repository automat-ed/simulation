#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "webots/Keyboard.hpp"
#include "webots/Supervisor.hpp"

namespace AutomatED {

class KeyboardController {
public:
  KeyboardController(webots::Supervisor *webots_supervisor,
                     ros::NodeHandle *ros_handle);
  ~KeyboardController();

  void keyLoop();

private:
  // ROS Handler
  ros::NodeHandle *nh_;

  // ROS parameters
  double l_scale_;
  double a_scale_;
  int time_step_;
  int frequency_;

  // Control values
  double linear_;
  double angular_;

  // ROS Publisher
  ros::Publisher cmd_pub_;

  // Webots Keyboard
  webots::Keyboard *keyboard_;
  int prev_key;
};

} // namespace AutomatED