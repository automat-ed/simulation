#include "utils/GroundTruthPose.hpp"

using namespace AutomatED;

GroundTruthPose::GroundTruthPose(webots::Supervisor *webots_supervisor, ros::NodeHandle *ros_handle)
{
    // Handlers
    wb_ = webots_supervisor;
    nh_ = ros_handle;

    // ROS Parameters
    nh_->param<std::string>("ground_truth_pose/robot_name", robot_name_, "StreetSmart");
    nh_->param("ground_truth_pose/frequency", frequency_, 30);
    nh_->param<std::string>("ground_truth_pose/frame_id", frame_id_, "map");

    // ROS Publisher
    pose_pub_ = nh_->advertise<nav_msgs::Odometry>("/ground_truth/pose", 1);

    // Get starting position of robot
    robot_ = wb_->getSelf();
    start_pos_ = new double[3];
    start_pos_[0] = robot_->getPosition()[0];
    start_pos_[1] = robot_->getPosition()[1];
    start_pos_[2] = robot_->getPosition()[2];
}

GroundTruthPose::~GroundTruthPose()
{
    pose_pub_.shutdown();
}

void GroundTruthPose::publishPose()
{
    // Get position of robot from starting position
    double *cur_pos = new double[3];
    cur_pos[0] = robot_->getPosition()[0] - start_pos_[0];
    cur_pos[1] = robot_->getPosition()[1] - start_pos_[1];
    cur_pos[2] = robot_->getPosition()[2] - start_pos_[2];

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.pose.pose.position.x = cur_pos[2];
    msg.pose.pose.position.y = cur_pos[0];
    msg.pose.pose.position.z = cur_pos[1];

    pose_pub_.publish(msg);
}