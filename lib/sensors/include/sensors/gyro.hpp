#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Gyro.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED
{

  class Gyro
  {
  public:
    Gyro(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle);
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
    std::string frame_id;
    int sampling_period;
    std::string ground_truth_topic;
    std::string noise_topic;
    double noise_mean;
    double noise_std;
    int noise_seed;

    // ROS publisher
    ros::Publisher ground_truth_pub;
    ros::Publisher noise_pub;

    // Tf2
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    void publishTF();

    // Noise
    std::mt19937 *gen;
    double gaussianNoise();
  };

} // namespace AutomatED
