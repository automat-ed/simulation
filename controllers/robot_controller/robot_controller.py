#!/usr/bin/env python3

"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import rospy
from custom_msgs.msg import ControlStamped
import math

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
front_right_led = robot.getDevice("front_right_led")
front_left_led = robot.getDevice("front_left_led")
rear_right_led = robot.getDevice("rear_right_led")
rear_left_led = robot.getDevice("rear_left_led")
# Distance between front and rear wheels
WHEEL_BASE_LENGTH = 46
CENTRE_TO_WHEEL_DISTANCE = 36
WHEEL_RADIUS = 12

def steer(angle_in_degrees):
    # The angle_in_degrees should be inside <-26,26> because of the steering limit
    # If angle_in_degrees is 0 then retrun wheels to default position
    if angle_in_degrees == 0:
        left_steer.setPosition(0)
        right_steer.setPosition(0)
        return    

    ideal_wheel_angle = math.radians(angle_in_degrees)
    ideal_curvature_radius = WHEEL_BASE_LENGTH / math.tan(ideal_wheel_angle)
    inside_wheel_angle = math.atan(WHEEL_BASE_LENGTH / (ideal_curvature_radius - CENTRE_TO_WHEEL_DISTANCE))
    outside_wheel_angle = math.atan(WHEEL_BASE_LENGTH / (ideal_curvature_radius + CENTRE_TO_WHEEL_DISTANCE))
    if ideal_wheel_angle >= 0:
        left_steer.setPosition(inside_wheel_angle)
        right_steer.setPosition(outside_wheel_angle)
    else:
        left_steer.setPosition(outside_wheel_angle)
        right_steer.setPosition(inside_wheel_angle)

def set_Velocity(speed, acceleration):
    angular_velocity = speed / WHEEL_RADIUS
    rear_right_wheel.setPosition(float('+inf'))
    rear_left_wheel.setPosition(float('+inf'))
    front_right_wheel.setPosition(float('+inf'))
    front_left_wheel.setPosition(float('+inf'))
    rear_right_wheel.setVelocity(angular_velocity)
    rear_left_wheel.setVelocity(angular_velocity)
    front_right_wheel.setVelocity(angular_velocity)
    front_left_wheel.setVelocity(angular_velocity)

def set_front_leds(value):
    front_left_led.set(value)
    front_right_led.set(value)

def set_rear_leds(value):
    rear_left_led.set(value)
    rear_right_led.set(value)    


# Turn the lights on
set_front_leds(1)
set_rear_leds(1)


class Control:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

    # Subscribers
        self.control_sub = rospy.Subscriber('/cmd', ControlStamped, self.receive_command, queue_size=10)

    # Publishers

    def receive_command(self, data):
        set_Velocity(data.speed, data.acceleration)
        steer(data.steering_angle)


# Instantiate ROS node
control = Control()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    pass
    

# Enter here exit cleanup code.
