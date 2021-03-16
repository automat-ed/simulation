#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/PositionSensor.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED
{

    class WheelOdom
    {
    public:
        WheelOdom(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle);
        ~WheelOdom();

        void publishWheelOdom();

    private:
        // Handlers
        webots::Supervisor *wb;
        ros::NodeHandle *nh;

        // Webots devices
        webots::PositionSensor *rr_odom;
        webots::PositionSensor *rl_odom;
        webots::PositionSensor *fr_odom;
        webots::PositionSensor *fl_odom;

        // Wheel position
        double prev_r_pos;
        double prev_l_pos;
        ros::Time prev_time;

        // ROS parameters
        std::string rr_name;
        std::string rl_name;
        std::string fr_name;
        std::string fl_name;
        std::string frame_id;
        int sampling_period;
        std::string ground_truth_topic;
        std::string noise_topic;
        double noise_mean;
        double noise_std;
        int noise_seed;
        double wheel_separation;
        double wheel_radius;

        // ROS publisher
        ros::Publisher ground_truth_pub;
        ros::Publisher noise_pub;

        // Noise
        std::mt19937 *gen;
        double gaussianNoise();

        // Helpers
        double calcLinearVelocity(double r_pos, double l_pos, ros::Time curr_time);
    };

} // namespace AutomatED
