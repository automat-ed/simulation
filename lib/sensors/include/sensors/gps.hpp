#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/GPS.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED
{

  class GPS
  {
  public:
    GPS(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle);
    ~GPS();

    void publishGPSCoordinate();

  private:
    webots::Supervisor *wb;
    ros::NodeHandle *nh;

    // Webots Devices
    webots::GPS *gps;

    // ROS parameters
    std::string gps_name;
    std::string frame_id;
    int sampling_period;
    std::string gt_coordinate_topic;
    std::string coordinate_topic;
    double noise_mean;
    double noise_std;
    int noise_seed;

    // ROS publishers
    ros::Publisher gt_coordinate_pub;
    ros::Publisher coordinate_pub;

    // Tf2
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    void publishTF();

    // Noise
    std::mt19937 *gen;
    double gaussianNoise();
  };

} // namespace AutomatED
