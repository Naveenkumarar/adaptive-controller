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
import csv 

import numpy as np
import roslib
import rospy
import ros_numpy

from geometry_msgs.msg import Vector3Stamped,Twist

import cv2

from qcar.product_QCar import QCar
from qcar.q_essential import Camera2D
from std_msgs.msg       import Float64


class MinimumJerk():
    def __init__(self):
        self.current_value = 0
        self.current_time = 0
        self.sampling_time = 100
        self.next_value = 0

    def update(self):
        tau = self.current_time / (self.current_time+self.sampling_time)
        cal = (6*tau**5) - (15*tau**4) + (10*tau**3)
        self.next_value = self.current_value / cal
        return self.next_value

    def update_current_value(self,current_value):
        self.current_value = current_value
        
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
        self.dt = self.loop_sample_time
        self.longitudinal_car_speed = 0.
        self.force_feeback = 0.
        self.longitudinal_velocity_pub = rospy.Publisher('/adaptive_response', Vector3Stamped, queue_size=10)
        self.direction=1
        #adaptive model parameter
        self.b=8
        self.command = np.array([self.throttle, self.steering])

        self.init_time = rospy.Time.now()
        self.filter_time = rospy.Time.now()
        self.prev_steer = 0

    def force_calculation(self):
        steer = self.command[1]
        diff = - steer
        if diff > 0 : dir=1
        else : dir =-1

        autocenter_control_p = 0.5
        autocenter_control_d = 1.5
        wheel_resistance = 0.5
        torque = (autocenter_control_p*diff)+(autocenter_control_d*(steer - self.prev_steer))-(wheel_resistance*steer)
        self.force_feeback = min(abs(torque),1) * dir

        self.prev_steer = steer

    def loop(self):
        self.last_time = rospy.Time.now()
        while not rospy.is_shutdown():  
                   
            ## write commands
            self.command = np.array([self.throttle, self.steering])
            self.command[0] = (-1/(2*self.b))*self.longitudinal_car_speed + self.command[0] / 2
            self.command[1] = (-1/(2*self.b))*self.force_feeback + self.command[1] / 2
            LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            if self.steering > 0.3:
                LEDs[0] = 1
                LEDs[2] = 1
            elif self.steering < -0.3:
                LEDs[1] = 1
                LEDs[3] = 1
            if self.steering < 0:
                LEDs[5] = 1

            print("Motor command",str(self.command),"---- velocity",str(self.longitudinal_car_speed),"---- ff",str(self.force_feeback))
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()
            self.filter_time = (self.current_time - self.init_time).to_sec()
            # read motor current, battery voltage and encoder counts
            motor_current,  batteryVoltage, encoder_count = self.myCar.read_write_std(self.command, LEDs)
            encoder_speed = (encoder_count - self.last_encoder_count) / self.dt  # differentiate encoder counts
            self.longitudinal_car_speed = self.basic_speed_estimation(encoder_speed)
            # self.force_feeback = self.kc*motor_current - self.b*encoder_speed* (1/720/4)  *  ( (13*19) / (70*37) )  *  1   *  2*np.pi  *  0.0342 
            
            self.last_time = self.current_time
            self.last_encoder_count = encoder_count
            self.force_calculation()   
            self.velocity_feedback()


    def velocity_feedback(self):
        us_velocity = self.longitudinal_car_speed / (2*self.b)**0.5
        us_ff = self.force_feeback / (2*self.b)**0.5
        adaptive_command = Vector3Stamped()
        adaptive_command.header.stamp = rospy.Time.now()
        adaptive_command.header.frame_id = 'adaptive response'
        adaptive_command.vector.x = float(us_velocity)
        adaptive_command.vector.y = float(us_ff)
        adaptive_command.vector.z = float(0.0)
        self.longitudinal_velocity_pub.publish(adaptive_command)

    def basic_speed_estimation(self,mtr_speed):

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
            self.direction = 1
        if data.vector.z < -0.5:
            self.throttle = -1.0 * data.vector.y
            self.direction = -1
        print(self.steering , self.throttle)

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
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

