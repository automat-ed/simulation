#include "ros/ros.h"
#include "webots/Motor.hpp"
#include "webots/Supervisor.hpp"
#include <random>

namespace AutomatED
{

  class WheelOdometry
  {
  public:
    WheelOdometry(webots::Supervisor *webots_supervisor,
                  ros::NodeHandle *ros_handle);
    ~WheelOdometry();

    void publishWheelOdometry();

  private:
    // Handlers
    webots::Supervisor *wb;
    ros::NodeHandle *nh;

    // Webots devices
    webots::Motor *wheels[4];

    // ROS parameters
    std::string frame_id;
    int sampling_period;
    std::string ground_truth_topic;
    std::string noise_topic;
    double wheel_separation;
    double wheel_radius;
    double noise_mean;
    double noise_std;
    double bias_mean;
    double bias_std;
    int noise_seed;

    // ROS publisher
    ros::Publisher ground_truth_pub;
    ros::Publisher noise_pub;

    // Noise
    std::mt19937 *gen;
    double gaussianNoise();
    double bias;

    // Helper functions
    void getLocalRotationalVelocity(webots::Node *solid, double *rvel_local);
    void transposeOrientation(const double *matrix, double *matrix_t);
    void transformVelocity(const double *matrix, const double *vector,
                           double *new_vec);
  };

} // namespace AutomatED
