#include "sensors/wheelOdom.hpp"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "webots/PositionSensor.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

WheelOdom::WheelOdom(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle)
{
    wb = webots_supervisor;
    nh = ros_handle;

    // Get ROS parameters
    nh->param<std::string>("wheel_odom/rear_right_name", rr_name, "rr_odom");
    nh->param<std::string>("wheel_odom/rear_left_name", rl_name, "rl_odom");
    nh->param<std::string>("wheel_odom/front_right_name", fr_name, "fr_odom");
    nh->param<std::string>("wheel_odom/front_left_name", fl_name, "fl_odom");
    nh->param<std::string>("wheel_odom/frame_id", frame_id, "base_link");
    nh->param("wheel_odom/sampling_period", sampling_period, 32);
    nh->param<std::string>("wheel_odom/ground_truth_topic", ground_truth_topic, "/wheel_odom/ground_truth");
    nh->param<std::string>("wheel_odom/noise_topic", noise_topic, "/wheel_odom/data");
    nh->param("wheel_odom/noise_mean", noise_mean, 0.0);
    nh->param("wheel_odom/noise_std", noise_std, 0.002);
    nh->param<int>("wheel_odom/noise_seed", noise_seed, 17);
    nh->param("wheel_separation", wheel_separation, 0.6);
    nh->param("wheel_radius", wheel_radius, 0.12);

    // Create publishers
    ground_truth_pub = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>(ground_truth_topic, 1);
    noise_pub = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>(noise_topic, 1);

    // Get wheel position sensors
    rr_odom = wb->getPositionSensor(rr_name);
    rl_odom = wb->getPositionSensor(rl_name);
    fr_odom = wb->getPositionSensor(fr_name);
    fl_odom = wb->getPositionSensor(fl_name);

    // Enable wheel position sensors
    rr_odom->enable(sampling_period);
    rl_odom->enable(sampling_period);
    fr_odom->enable(sampling_period);
    fl_odom->enable(sampling_period);

    // Initalize position values
    prev_r_pos = 0;
    prev_l_pos = 0;
    prev_noisy_r_pos = 0;
    prev_noisy_l_pos = 0;
    prev_time = ros::Time::now();

    // Initialize generator with seed
    gen = new std::mt19937{(long unsigned int)noise_seed};
}

WheelOdom::~WheelOdom()
{
    noise_pub.shutdown();
    ground_truth_pub.shutdown();

    // Disable each position sensor
    rr_odom->disable();
    rl_odom->disable();
    fr_odom->disable();
    fl_odom->disable();
}

void WheelOdom::publishWheelOdom()
{
    // Get time
    ros::Time curr_time = ros::Time::now();

    // Get readings from position sensors (assumes 4)
    double rr_pos = rr_odom->getValue();
    double rl_pos = rl_odom->getValue();
    double fr_pos = fr_odom->getValue();
    double fl_pos = fl_odom->getValue();

    // Take average of each side
    double r_pos = (rr_pos + fr_pos) / 2;
    double l_pos = (rl_pos + fl_pos) / 2;

    // Calculate change in angle
    double dr = r_pos - prev_r_pos;
    double dl = l_pos - prev_l_pos;

    // Calculate linear velocity from positions
    double robot_vel = calcLinearVelocity(dr, dl, curr_time);

    // Publish ground truth (which is still kinda noisy)
    geometry_msgs::TwistWithCovarianceStamped gt;
    gt.header.stamp = ros::Time::now();
    gt.header.frame_id = "base_link";
    gt.twist.twist.linear.x = robot_vel;
    ground_truth_pub.publish(gt);

    // Add noise to readings
    double noisy_rr_pos = rr_pos + gaussianNoise();
    double noisy_rl_pos = rl_pos + gaussianNoise();
    double noisy_fr_pos = fr_pos + gaussianNoise();
    double noisy_fl_pos = fl_pos + gaussianNoise();

    // Take average of each side
    double noisy_r_pos = (noisy_rr_pos + noisy_fr_pos) / 2;
    double noisy_l_pos = (noisy_rl_pos + noisy_fl_pos) / 2;

    // Calculate change in angle
    double noisy_dr = noisy_r_pos - prev_noisy_r_pos;
    double noisy_dl = noisy_l_pos - prev_noisy_l_pos;

    // Calculate linear velocity from positions
    double noisy_robot_vel = calcLinearVelocity(noisy_dr, noisy_dl, curr_time);

    // Publish noisy data (which even more noisy than gt)
    geometry_msgs::TwistWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.twist.twist.linear.x = noisy_robot_vel;
    msg.twist.covariance[0] = std::pow(noise_mean + noise_std, 2);
    noise_pub.publish(msg);

    // Save current state for next time
    prev_r_pos = r_pos;
    prev_l_pos = l_pos;
    prev_noisy_r_pos = noisy_r_pos;
    prev_noisy_l_pos = noisy_l_pos;
    prev_time = curr_time;
}

double WheelOdom::calcLinearVelocity(double dr, double dl, ros::Time curr_time)
{
    // Calculate average angular velocity between two consecutive time stamps
    double r_avel = dr / (curr_time - prev_time).toSec();
    double l_avel = dl / (curr_time - prev_time).toSec();

    // Convert to linear velocity
    double r_vel = r_avel * wheel_radius;
    double l_vel = l_avel * wheel_radius;

    // Calculate linear velocity of robot
    return (l_vel + r_vel) / 2;
}

double WheelOdom::gaussianNoise()
{
    std::normal_distribution<double> d{noise_mean, noise_std};
    return d(*gen);
}