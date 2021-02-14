#!/usr/bin/env python3

"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import numpy as np
import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from custom_msgs.msg import ControlStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

from controller import Robot
from controller import Lidar



class Control:
    def __init__(self):
    
        self.robot = Robot()
 
        # Devices
        self.lidar = self.setup_lidar('Velodyne VLP-16', frequency=10)
        self.rear_right_wheel = self.setup_wheel('rear_right_wheel', speed=0)
        self.rear_left_wheel = self.setup_wheel('rear_left_wheel', speed=0)
        self.front_right_wheel = self.setup_wheel('front_right_wheel', speed=0)
        self.front_left_wheel = self.setup_wheel('front_left_wheel', speed=0)
        
      
        
        # ROS
        rospy.init_node('robot_controller', anonymous=True)
        
     
        # Subscribers
            #self.control_sub = rospy.Subscriber('/cmd', ControlStamped, self.receive_command, queue_size=10)
    
        # Publishers
            #self.camera_pub = rospy.Publisher('/camera', Image, queue_size=10)
    
        self.point_cloud_pub = rospy.Publisher('/lidar/point_cloud', PointCloud, queue_size=10)
       
       
    def setup_lidar(self, device_name, frequency):
        lidar = self.robot.getDevice(device_name)
        lidar.enable(frequency)
        lidar.enablePointCloud()
        return lidar
        
    def setup_wheel(self, device_name, speed):
        wheel = self.robot.getDevice(device_name)
        wheel.setPosition(float('+inf'))
        wheel.setVelocity(speed)
        return wheel
        
        
    def create_point_cloud_msg(self, point_cloud):
        cloud = PointCloud()
        cloud.header.stamp = rospy.get_rostime()
        cloud.header.frame_id = 'lidar_frame' 
        layerChannel = ChannelFloat32()
        layerChannel.name = 'layer'
        for i in range(self.lidar.getNumberOfPoints()):
            point = Point32()
            point.x = point_cloud[i].x
            point.y = point_cloud[i].y
            point.z = point_cloud[i].z
            cloud.points.append(point)
            layerChannel.values.append(point_cloud[i].layer_id)
        cloud.channels.append(layerChannel)
        return cloud
     
    def run(self):
         timestep = int(self.robot.getBasicTimeStep())
         while self.robot.step(timestep) != -1:
         
             # Get Lidar readings
             point_cloud = self.lidar.getPointCloud()
             point_cloud_msg = self.create_point_cloud_msg(point_cloud)
             self.point_cloud_pub.publish(point_cloud_msg)



control = Control()
control.run()
    
    
    
    
    
    
    # Publish camera image
    #raw_image = kinect_camera.getImage()
    #img = np.frombuffer(raw_image, dtype=np.uint8).reshape((kinect_camera.getHeight(), kinect_camera.getWidth(), -1))
    #img_msg = control.bridge.cv2_to_imgmsg(img)
    #control.camera_pub.publish(img_msg)
    
    
    


