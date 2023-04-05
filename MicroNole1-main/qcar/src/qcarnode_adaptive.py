#!/usr/bin/env python3

"""
Publishes the following:
1. publish camera image to (192.168.2.19)

Subscribes to the following:
1. subscribe joystick command from (192.168.2.19)
2. 

Functions:
1. write command to MicroNole
"""

from __future__ import division, print_function, absolute_import

import time
import sys

import numpy as np
import roslib
import rospy
import ros_numpy

from geometry_msgs.msg import Vector3Stamped,Twist

import cv2

from qcar.product_QCar import QCar
from qcar.q_essential import Camera2D


class RacingNode(object):
    def __init__(self, publish_rate = 480.0):
        super().__init__()

        self.command_sub = rospy.Subscriber('/racing_cockpit/ctrl_cmd', Vector3Stamped, self.racing_command, queue_size=10)
        self.loop_rate = publish_rate  #
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

        # initialization of variables
        self.steering, self.throttle = 0.0, 0.0
        self.myCar = QCar()

         # odometry variables
        self.last_encoder_count = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.dt = self.sample_time
        self.longitudinal_car_speed = 0.
        self.force_feeback = 0.
        self.longitudinal_velocity_pub = rospy.Publisher('/adaptive_response', Twist, queue_size=10)

        #adaptive model parameter
        self.b=0.1
        self.kc=0.1
        self.k2=0.1
        self.k4=0.1
        self.gamma=0.1
        self.H=0.1
        self.delta=0.1

    def loop(self):
        self.last_time = rospy.Time.now()
        while not rospy.is_shutdown():            
            ## write commands
            self.command = np.array([self.throttle, self.steering])
            LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            if self.steering > 0.3:
                LEDs[0] = 1
                LEDs[2] = 1
            elif self.steering < -0.3:
                LEDs[1] = 1
                LEDs[3] = 1
            if self.steering < 0:
                LEDs[5] = 1

            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()

            if self.use_filter:
                self.filter_time = (self.current_time - self.init_time).to_sec()

            # read motor current, battery voltage and encoder counts
            motor_current, battery_voltage, encoder_count = self.myCar.read_std()
            encoder_speed = (encoder_count - self.last_encoder_count) / self.dt  # differentiate encoder counts
            self.longitudinal_car_speed = self.basic_speed_estimation(encoder_speed)


            self.force_feeback = self.kc*motor_current - self.b*encoder_count

            self.adaptive_model()

            self.last_time = self.current_time
            self.last_encoder_count = encoder_count
            self.rate.sleep()

    def adaptive_model(self):
        velocity_modified = self.longitudinal_car_speed - self.k2*self.force_feeback
        force_modified = self.force_feeback + self.k4 * self.longitudinal_car_speed
        self.k4+= - self.gamma * self.H * self.longitudinal_car_speed * self.delta
        self.k2+= - self.gamma * self.H * self.force_feeback * self.delta
        adaptive_cmd = Twist()
        adaptive_cmd.linear.x = float(velocity_modified)
        adaptive_cmd.linear.y = float(force_modified)
        adaptive_cmd.vector.z = 0.0
        adaptive_cmd.angular.x = float(self.k4)
        adaptive_cmd.angular.y = float(self.k2)
        adaptive_cmd.angular.z = 0.0
        self.longitudinal_velocity_pub.publish(adaptive_cmd)

    def basic_speed_estimation(self,mtr_speed):
        '''This function contains the out-of-the-box mapping from encoder counts/s to the longitudonal 
        speed of the QCar. 

        Inputs:
        mtr_speed - encoder speed in counts/s
        
        Outputs:
        car_speed - longitudonal car speed in m/s'''

        # term 1 - counts/s to rot/s 
        # term 2 - ( diff pinion * pinion )  /  ( spur * diff spur )
        # term 3 - shaft to axle gain 
        # term 4 - rot/s to rad/s 
        # term 5 - wheel radius 

        #		  term 1            term 2            term 3   term 4      term 5
        return  (1/720/4)  *  ( (13*19) / (70*37) )  *  1   *  2*np.pi  *  0.0342  *  mtr_speed

    # --------------------------------------------------------------------------------------------
    def racing_command(self,data):
        self.steering = data.vector.x
        self.throttle = 0.0
        if data.vector.z > 0.5:
            self.throttle = data.vector.y
        if data.vector.z < -0.5:
            self.throttle = -1.0 * data.vector.y

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
        # perform shutdown tasks here
        command = np.array([0.0, 0.0])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        current, batteryVoltage, encoderCounts = self.myCar.read_write_std(command, LEDs)
        rospy.loginfo("Shutting down cleanly...")


if __name__ == '__main__':
    rospy.init_node('racingNode')
    racing = RacingNode()
    rospy.on_shutdown(racing.shutdown)

    try:
        racing.loop()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')

