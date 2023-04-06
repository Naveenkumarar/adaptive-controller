#!/usr/bin/env python3

"""
Publishes the following:
1. publish requried joystick command to MicroNole

Subscribes to the following:
1. subscribe racing cockpit command
2. subscribe camera infor from MicroNole (192.168.2.15)

Functions:
1. display the camera view
"""

import time

import numpy as np
import math
import rospy

from geometry_msgs.msg import Vector3Stamped,Twist
from sensor_msgs.msg import Joy, Image, CompressedImage
import cv2
import pickle
   


class racingNode(object):
    """docstring for ClassName"""

    def __init__(self, publish_joy=True, publish_rate= 120.):
        """Constructor for racing simulator"""
        super().__init__()

        self.publish_rate = publish_rate  # /G29/joy topic miximal rate: 35hz
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        self.ctrl_pub = rospy.Publisher('/racing_cockpit/ctrl_cmd', Vector3Stamped, queue_size=10)
        self.ctrl_sub = rospy.Subscriber("/G29/joy", Joy, self.ctrl_callback, queue_size=10)
        self.adaptive_sub = rospy.Subscriber("/adaptive_response", Twist, self.adaptive_callback, queue_size=10)
        self.delay = 0.0
        
        self.axes = np.zeros(6)
        self.button = np.zeros(18)
        self.command = np.zeros(3)
        self.throttle_scale, self.steering_scale = 0.2/2, math.pi/2

        #passivity constant
        self.k1 = 0.1
        self.k2 = 0.1
        self.k3 = 0.3
        self.k4 = 0.4
        self.velocity = 0
        self.forcefeedback = 0

    def publish_data(self):
        while not rospy.is_shutdown():
            # all racing cockpit joystick command
            self.steering = self.axes[0]     # +:left, -:right [-1,1]
            self.throttle = self.axes[1]+1   # [-1,1] --> [0,2] default:-1
            self.brake    = self.axes[2]+1   # [-1,1] --> [0,2] default:-1
            self.clutch   = self.axes[3]     # [-1,1]
            
            self.A, self.B, self.X, self.Y = self.button[:4]
            self.shift_paddle_right = self.button[4]
            self.shift_paddle_left = self.button[5]
            self.RSB = self.button[8]
            self.LSB = self.button[9]
            self.xbox = self.button[10]
            self.gearshift_left_f = self.button[12]
            self.gearshift_middle_f = self.button[14]
            self.gearshift_right_f = self.button[16]
            self.gearshift_left_b = self.button[13]
            self.gearshift_middle_b = self.button[15]
            self.gearshift_right_b = self.button[17]
            
            self.command[0] = np.maximum(np.minimum(self.steering_scale * self.steering,math.pi/6),-math.pi/6)
            self.command[1] = np.maximum(self.throttle_scale * ( np.abs(self.throttle) - np.abs(self.brake) ), 0)
            self.command[2] = 0
            if self.gearshift_left_f==1 or self.gearshift_middle_f==1 or self.gearshift_right_f==1:
                self.command[2] = 1
            if self.gearshift_left_b==1 or self.gearshift_middle_b==1 or self.gearshift_right_b==1:
                self.command[2] = -1
            self.modify_input()

    def modify_input(self):
        self.command[0]=self.command[0] - self.k1 * self.velocity - self.k2 * self.forcefeedback
        self.command[1]=self.command[1] - self.k3 * self.velocity - self.k4 * self.forcefeedback
        self.publisher_joy()

    def ctrl_callback(self, data):
        self.axes = data.axes
        self.button = data.buttons
        
    def adaptive_callback(self,data):
        self.k4 = data.angular.x
        self.k2 = data.angular.y
        self.velocity = data.vector.x
        self.forcefeedback = data.vector.y

    def publisher_joy(self):
        joy_command = Vector3Stamped()
        joy_command.header.stamp = rospy.Time.now()
        joy_command.header.frame_id = 'remote racing'
        joy_command.vector.x = float(self.command[0])
        joy_command.vector.y = float(self.command[1])
        joy_command.vector.z = float(self.command[2])
        self.ctrl_pub.publish(joy_command)

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
        
        rospy.loginfo("Shutting down cleanly...")
        
    def timer_watcher(self, event):
        rospy.loginfo(f"Delay: {self.delay}")


if __name__ == '__main__':
    rospy.init_node('racingNode')
    racing = racingNode()
    rospy.on_shutdown(racing.shutdown)

    try:
        racing.publish_data()
        # rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')



