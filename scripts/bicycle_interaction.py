#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from autonomous_bicycle.cfg import bicycle_interactionConfig

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0


class BicycleInteraction:
    def __init__(self):
        self.pub_torque_wheel = rospy.Publisher('/bycycle_interaction/torque_wheel', Float32, queue_size=1)
        self.pub_vel_steering = rospy.Publisher('/bycycle_interaction/vel_steering', Float32, queue_size=1)

        self.wheelTorque = Float32()
        self.steeringVelocity = Float32()
        self.torque_wheel = 0
        self.vel_steering = 0
        self.enable_bicycle = True

        self.rate = rospy.get_param('~rate', 3.0)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.30)
        self.angle_limit = rospy.get_param('~angle_limit', 50)
        self.imu_angle = rospy.get_param('~imu_angle', '/imu')

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.srv = Server(bicycle_interactionConfig, self.reconfig_callback)  # define dynamic_reconfigure callback

        self.sub = rospy.Subscriber(self.imu_angle, Imu, self.process_imu_message_angle, queue_size=1)
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            print(11111)
            self.publish()
            rate.sleep()

    def reconfig_callback(self, config, level):
        self.torque_wheel = config['torque_wheel']
        self.vel_steering = config['vel_steering']
        return config

    def process_imu_message_angle(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        self.roll = self.roll * rad2degrees
        self.pitch = self.pitch * rad2degrees
        self.yaw = self.yaw * rad2degrees

        # if bicycle fall down, shutdown motor
        if abs(self.pitch) > self.angle_limit:
            self.enable_bicycle = False
        else:
            self.enable_bicycle = True

    def publish(self):
        # send angular velocity to wheels.

        if self.enable_bicycle or self.torque_wheel == -1:
            self.wheelTorque = self.torque_wheel
        else:
            self.wheelTorque = 0

        self.steeringVelocity = self.vel_steering

        self.pub_torque_wheel.publish(self.wheelTorque)
        self.pub_vel_steering.publish(self.steeringVelocity)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('bicycle_interaction_helper')
    try:
        obj = BicycleInteraction()
    except:
        pass