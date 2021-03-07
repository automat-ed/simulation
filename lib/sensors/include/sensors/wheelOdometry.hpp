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

    // Noise
    std::mt19937 *gen;
    double linearGaussianNoise();
    double angularGaussianNoise();
    double linear_bias;
    double angular_bias;

    // Helper functions
    void getLocalRotationalVelocity(webots::Node *solid, double *rvel_local);
    void transposeOrientation(const double *matrix, double *matrix_t);
    void transformVelocity(const double *matrix, const double *vector,
                           double *new_vec);
  };

} // namespace AutomatED
