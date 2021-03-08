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
    double linear_vel;
    double angular_vel;

    // ROS Parameters
    std::string frame_id;
    std::string ground_truth_topic;
    std::string noise_topic;
    double wheel_separation;
    double wheel_radius;
    double linear_mean;
    double linear_std;
    double linear_bias_mean;
    double linear_bias_std;
    double angular_mean;
    double angular_std;
    double angular_bias_mean;
    double angular_bias_std;
    int noise_seed;

    // ROS publisher
    ros::Publisher ground_truth_pub;
    ros::Publisher noise_pub;

    // ROS Subscriber
    ros::Subscriber cmd_vel_sub;

    // Noise
    std::mt19937 *gen;
    double linearGaussianNoise();
    double angularGaussianNoise();
    double linear_bias;
    double angular_bias;

    void velocityCallback(const geometry_msgs::TwistConstPtr &cmd);
    void stopMotors();
  };

} // namespace AutomatED
