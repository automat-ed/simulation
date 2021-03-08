#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/Accelerometer.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED
{

  class Accelerometer
  {
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
    std::string frame_id;
    int sampling_period;
    std::string ground_truth_topic;
    std::string noise_topic;
    double noise_mean;
    double noise_std;
    double bias_mean;
    double bias_std;
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
    double bias;
  };

} // namespace AutomatED
