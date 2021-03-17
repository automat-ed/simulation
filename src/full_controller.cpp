#include "ros/ros.h"
#include "sensors/accelerometer.hpp"
#include "sensors/gps.hpp"
#include "sensors/gyro.hpp"
#include "sensors/inertialUnit.hpp"
#include "sensors/lidar.hpp"
#include "sensors/wheelOdom.hpp"
#include "steering/diffSteering.hpp"
#include "utils/KeyboardController.hpp"
#include "webots/Motor.hpp"
#include "webots/LED.hpp"
#include "webots/Supervisor.hpp"

int main(int argc, char **argv)
{
  ROS_INFO("Initializing full_controller...");

  // Set up ROS node
  ros::init(argc, argv, "full_controller");
  ros::NodeHandle nh("~");

  // Get ROS parameters
  int step_size;
  nh.param("step_size", step_size, 32);
  bool use_keyboard_control;
  nh.param("use_keyboard_control", use_keyboard_control, false);

  // Set up Webots
  webots::Supervisor *supervisor = new webots::Supervisor();

  // Get webots motors
  std::vector<webots::Motor *> motors = {
      supervisor->getMotor("front_left_wheel"),
      supervisor->getMotor("front_right_wheel"),
      supervisor->getMotor("rear_left_wheel"),
      supervisor->getMotor("rear_right_wheel")};

  // Instantiate sensor wrappers
  AutomatED::Lidar lidar = AutomatED::Lidar(supervisor, &nh);
  AutomatED::GPS gps = AutomatED::GPS(supervisor, &nh);
  AutomatED::InertialUnit imu = AutomatED::InertialUnit(supervisor, &nh);
  AutomatED::Gyro gyro = AutomatED::Gyro(supervisor, &nh);
  AutomatED::Accelerometer accelerometer =
      AutomatED::Accelerometer(supervisor, &nh);
  AutomatED::WheelOdom wheel_odom = AutomatED::WheelOdom(supervisor, &nh);

  // Instantiate steering
  AutomatED::DiffSteering diffSteering = AutomatED::DiffSteering(motors, &nh);

  // Instaniate keyboard steering
  AutomatED::KeyboardController *keyboard;
  if (use_keyboard_control)
  {
    keyboard = new AutomatED::KeyboardController(supervisor, &nh);
  }

  // Turn on LEDs
  supervisor->getLED("front_left_led")->set(1);
  supervisor->getLED("front_right_led")->set(1);
  supervisor->getLED("rear_left_led")->set(1);
  supervisor->getLED("rear_right_led")->set(1);

  while (supervisor->step(step_size) != -1)
  {
    lidar.publishLaserScan();
    gps.publishGPSCoordinate();
    imu.publishImuQuaternion();
    gyro.publishGyro();
    accelerometer.publishAccelerometer();
    wheel_odom.publishWheelOdom();


    if (use_keyboard_control)
    {
      keyboard->keyLoop();
    }

    ros::spinOnce();
  }

  // Clean up
  delete supervisor;
  return 0;
}
