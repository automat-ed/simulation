#include "sensors/MultiSenseS21.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "webots/Camera.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

MultiSenseS21::MultiSenseS21(webots::Supervisor *webots_supervisor,
                             ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("camera/name", camera_name, "MultiSenseS21");
  nh->param<std::string>("camera/frame_id", frame_id, "camera");
  nh->param("camera/sampling_period", sampling_period, 32);

  // Create publishers
  ground_truth_pub = nh->advertise<sensor_msgs::Image>("/camera/ground_truth", 1);
  
  // Setup camera device
  camera = wb->getCamera(camera_name);
  camera->enable(sampling_period);
}

MultiSenseS21::~MultiSenseS21()
{
  ground_truth_pub.shutdown();
  camera->disable();
}

// get image from the Camera and publish it
void MultiSenseS21::publishCamera() {
  const unsigned char *colorImage = camera->getImage();
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.height = camera->getHeight();
  image.width = camera->getWidth();
  image.encoding = sensor_msgs::image_encodings::BGRA8;
  // image.is_bigendian = 0;
  image.step = sizeof(char) * 4 * camera->getWidth();

  image.data.resize(4 * camera->getWidth() * camera->getHeight());
  memcpy(&image.data[0], colorImage, sizeof(char) * 4 * camera->getWidth() * camera->getHeight());

  ground_truth_pub.publish(image);
}
