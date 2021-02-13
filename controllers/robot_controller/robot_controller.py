#!/usr/bin/env python3

"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from custom_msgs.msg import ControlStamped
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String, MultiArrayLayout
from cv_bridge import CvBridge
import cv2

import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

rear_right_wheel = robot.getDevice("rear_right_wheel")
rear_left_wheel = robot.getDevice("rear_left_wheel")
front_right_wheel = robot.getDevice("front_right_wheel")
front_left_wheel = robot.getDevice("front_left_wheel")
left_steer = robot.getDevice("left_steer")
right_steer = robot.getDevice("right_steer")


# Camera
kinect_camera = robot.getDevice("kinect color")
kinect_camera.enable(10)


class Control:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.bridge = CvBridge()
        self.speed = 0
        self.acceleration = 0
        self.steering_angle = 0
    # Subscribers
        self.control_sub = rospy.Subscriber('/cmd', ControlStamped, self.receive_command, queue_size=10)

    # Publishers
        self.camera_pub = rospy.Publisher('/camera', Image, queue_size=10)

    def receive_command(self, data):
        self.speed = data.control.speed
        self.acceleration = data.control.acceleration
        self.steering_angle = data.control.steering_angle


# Instantiate ROS node
control = Control()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    rear_right_wheel.setPosition(float('+inf'))
    rear_left_wheel.setPosition(float('+inf'))
    front_right_wheel.setPosition(float('+inf'))
    front_left_wheel.setPosition(float('+inf'))
    rear_right_wheel.setVelocity(control.speed)
    rear_left_wheel.setVelocity(control.speed)
    front_right_wheel.setVelocity(control.speed)
    front_left_wheel.setVelocity(control.speed)
    left_steer.setPosition(control.steering_angle)
    right_steer.setPosition(control.steering_angle)
    
    # Publish camera image
    raw_image = kinect_camera.getImage()
    img = np.frombuffer(raw_image, dtype=np.uint8).reshape((kinect_camera.getHeight(), kinect_camera.getWidth(), -1))
    img_msg = control.bridge.cv2_to_imgmsg(img)
    control.camera_pub.publish(img_msg)
