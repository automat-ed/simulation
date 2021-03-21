#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "webots/Supervisor.hpp"

namespace AutomatED
{

    class GroundTruthPose
    {
    public:
        GroundTruthPose(webots::Supervisor *webots_supervisor,
                        ros::NodeHandle *ros_handle);
        ~GroundTruthPose();

        void publishPose();

    private:
        // Handlers
        webots::Supervisor *wb_;
        ros::NodeHandle *nh_;

        // ROS Parameters
        std::string robot_name_;
        int frequency_;
        std::string frame_id_;

        // ROS Publisher
        ros::Publisher pose_pub_;

        // Robot parameters
        webots::Node *robot_;
        double *start_pos_;
    };

} // namespace AutomatED