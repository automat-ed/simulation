#include "ros/ros.h"
#include "webots/Camera.hpp"
#include "webots/RangeFinder.hpp"
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
    void publishRange();

  private:
    // Handlers
    webots::Supervisor *wb;
    ros::NodeHandle *nh;

    // Webots devices
    webots::Camera *camera;
    webots::RangeFinder *range;

    // ROS parameters
    std::string camera_name;
    std::string range_name;
    std::string frame_id;
    int sampling_period;

    // ROS publisher
    ros::Publisher camera_pub;
    ros::Publisher range_pub;
  };

} // namespace AutomatED
