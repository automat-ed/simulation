#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "webots/Motor.hpp"
#include <random>

namespace AutomatED
{

  class DiffSteering
  {
  public:
    DiffSteering(std::vector<webots::Motor *> motors,
                 ros::NodeHandle *ros_handle);
    ~DiffSteering();

    void publish();

  private:
    std::vector<webots::Motor *> wheels;
    ros::NodeHandle *nh;
    int wheel_count;

    // ROS Parameters
    std::string frame_id;
    std::string ground_truth_topic;
    std::string noise_topic;
    double wheel_separation;
    double wheel_radius;

    // ROS publisher
    ros::Publisher ground_truth_pub;
    ros::Publisher noise_pub;

    // ROS Subscriber
    ros::Subscriber cmd_vel_sub;

    void velocityCallback(const geometry_msgs::TwistConstPtr &cmd);
    void stopMotors();
  };

} // namespace AutomatED
