#include "ros/ros.h"
#include "webots/Camera.hpp"
#include "webots/Supervisor.hpp"

namespace AutomatED
{

  class MultiSenseS21
  {
  public:
    MultiSenseS21(webots::Supervisor *webots_supervisor,
                  ros::NodeHandle *ros_handle);
    ~MultiSenseS21();

    void publishCamera();

  private:
    // Handlers
    webots::Supervisor *wb;
    ros::NodeHandle *nh;

    // Webots devices
    webots::Camera *camera;

    // ROS parameters
    std::string camera_name;
    std::string frame_id;
    int sampling_period;

    // ROS publisher
    ros::Publisher ground_truth_pub;
  };

} // namespace AutomatED
