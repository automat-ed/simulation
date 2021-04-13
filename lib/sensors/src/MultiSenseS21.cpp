#include "sensors/MultiSenseS21.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "webots/Camera.hpp"
#include "webots/RangeFinder.hpp"
#include "webots/Supervisor.hpp"
#include <random>

using namespace AutomatED;

MultiSenseS21::MultiSenseS21(webots::Supervisor *webots_supervisor,
                             ros::NodeHandle *ros_handle)
{
  wb = webots_supervisor;
  nh = ros_handle;

  // Get ROS parameters
  nh->param<std::string>("multi_sense/camera/name", camera_name, "MultiSense S21 meta camera");
  nh->param<std::string>("multi_sense/range/name", range_name, "MultiSense S21 meta range finder");
  nh->param<std::string>("multi_sense/frame_id", frame_id, "multi_sense");
  nh->param("multi_sense/sampling_period", sampling_period, 32);

  // Create publishers
  camera_pub = nh->advertise<sensor_msgs::Image>("/multi_sense/rgb/data", 1);
  range_pub = nh->advertise<sensor_msgs::Image>("/multi_sense/range/data", 1);

  // Setup camera device
  camera = wb->getCamera(camera_name);
  camera->enable(sampling_period);

  // Setup rangeFinder device
  range = wb->getRangeFinder(range_name);
  range->enable(sampling_period);
}

MultiSenseS21::~MultiSenseS21()
{
  camera_pub.shutdown();
  range_pub.shutdown();
  camera->disable();
}

void MultiSenseS21::publishCamera()
{
  // Get image from Camera
  const unsigned char *colorImage = camera->getImage();

  // Construct Image message
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.height = camera->getHeight();
  image.width = camera->getWidth();
  image.encoding = sensor_msgs::image_encodings::BGRA8;
  // image.is_bigendian = 0;
  image.step = sizeof(char) * 4 * camera->getWidth();

  // Copy image into message
  image.data.resize(4 * camera->getWidth() * camera->getHeight());
  memcpy(&image.data[0], colorImage, sizeof(char) * 4 * camera->getWidth() * camera->getHeight());

  // Publish message
  camera_pub.publish(image);
}

void MultiSenseS21::publishRange()
{
  // Get range image from RangeFinder
  const float *rangeImageVector = range->getRangeImage();

  // Construct Image message
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = frame_id;
  image.height = range->getHeight();
  image.width = range->getWidth();
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = sizeof(float) * range->getWidth();

  // Copy image into message
  image.data.resize(sizeof(float) * range->getWidth() * range->getHeight());
  memcpy(&image.data[0], rangeImageVector, sizeof(float) * range->getWidth() * range->getHeight());

  range_pub.publish(image);
}