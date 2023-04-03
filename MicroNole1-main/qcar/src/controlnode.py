#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import
import sys

import numpy as np
import math
import roslib
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from qcar.msg import FilterData


class ControlNode(object):
    def __init__(self, publish_filter=True):
        super().__init__()

        # subscribe filter results from filternode
        self.sensor_hub = rospy.Subscriber('/qcar/filter_data', FilterData, self.receiving, queue_size=100)
        
        # subscribe lateral distance from csi_lane_error_node
        self.vision_pub = rospy.Subscriber('lane_detection/error/value', Float64, self.lane_error_sub, queue_size=100)

        # initialize publisher
        self.cmd_pub_ = rospy.Publisher('/qcar/auto_command', Vector3Stamped, queue_size=100)

        # controllers parameters
        self.speed_diff_last = 0.
        self.throttle_cmd_last = 0.
        
        self.desired_longitudinal_speed = 1.
        self.desired_yaw_rate = 0.
        
        self.Kp1 = 0.1               # for longitudinal speed control
        self.Ki1 = 0.16
        self.integral_state = 0.
        self.alpha = 0.01
        self.delta_k = 1

        self.sat_pwm = 0.3
        self.rate_limit = 0.001
        
        self.Kp2 = 0.1               # for yaw rate control
        self.Ki2 = 0.
        
        self.Kp31 = 0.1
        self.Kp32 = 0.5
        self.Ki31 = 0.1
        
        # data storage cache
        self.command_cache = np.array([[0., 0.]])

        # initialize variables
        self.filtered_lateral_velocity, self.filtered_longitudinal_velocity, self.filtered_yaw_rate = 0., 0., 0.
        self.timestamp_last, self.timestamp = 0., 0.0001
        self.speed_diff, self.yaw_rate_diff = 0., 0.
        self.lane_error = 0.
        self.integral_state1, self.integral_state2 = 0., 0.

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = 100.0  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)
    
    def longitudinal_speed_control(self):
        self.speed_diff = self.desired_longitudinal_speed - self.filtered_longitudinal_velocity
        
        self.integral_state = self.alpha * self.integral_state + self.delta_k * self.speed_diff
        throttle_cmd = 0.1 * self.desired_longitudinal_speed + 3*(self.Kp1 * self.speed_diff + self.Ki1 * self.integral_state)
        #  0.1*self.desired_longitudinal_speed is a feedforward offset (from experiment)
          
        if abs(throttle_cmd - self.throttle_cmd_last) >  self.rate_limit * self.sat_pwm:   # rate saturation, for safety
            throttle_cmd = self.throttle_cmd_last + np.sign(throttle_cmd - self.throttle_cmd_last) * self.rate_limit * self.sat_pwm
            
        self.throttle_cmd_last = throttle_cmd
        return throttle_cmd
    
    def yaw_rate_control(self):
        self.yaw_rate_diff = self.desired_yaw_rate - self.filtered_yaw_rate
        steering_angle = self.Kp2 * self.yaw_rate_diff + 0.025  # 0.025 is a command offset (from experiment)
        
        if steering_angle > math.pi/6:   # for safety
            steering_angle = math.pi/6
        if steering_angle < -math.pi/6:
            steering_angle = -math.pi/6
            
        return steering_angle
    
    def lane_keeping_control(self):
        desired_lateral_speed = -0.1 * self.lane_error
        self.lateral_diff = desired_lateral_speed - self.filtered_lateral_velocity
        self.integral_state1 = self.alpha * self.integral_state1 + self.delta_k * self.lateral_diff
        
        self.speed_diff = self.desired_longitudinal_speed - self.filtered_longitudinal_velocity
        self.integral_state2 = self.alpha * self.integral_state2 + self.delta_k * self.speed_diff
        
        self.yaw_rate_diff = self.desired_yaw_rate - self.filtered_yaw_rate
        
        throttle_cmd = 0.1381 * self.desired_longitudinal_speed + 3*(self.Kp1 * self.speed_diff + self.Ki1 * self.integral_state + self.Kp31 * self.lateral_diff + self.Ki31 * self.integral_state1)
        
        steering_angle = self.Kp2 * self.yaw_rate_diff + self.Kp32 * self.lateral_diff + 0.025
        
        if abs(throttle_cmd - self.throttle_cmd_last) >  self.rate_limit * self.sat_pwm:   # rate saturation, for safety
            throttle_cmd = self.throttle_cmd_last + np.sign(throttle_cmd - self.throttle_cmd_last) * self.rate_limit * self.sat_pwm
            
        self.throttle_cmd_last = throttle_cmd
        
        if steering_angle > math.pi/6:   # for safety
            steering_angle = math.pi/6
        if steering_angle < -math.pi/6:
            steering_angle = -math.pi/6
            
        return throttle_cmd, steering_angle

    def run_controllers(self):
        while not rospy.is_shutdown():
            self.dt = self.timestamp - self.timestamp_last
            # rospy.loginfo(f'gap time: {self.dt}')
            # run controllers
            # throttle_cmd, steering_angle =self.lane_keeping_control()
            
            throttle_cmd   = self.longitudinal_speed_control()
            steering_angle = self.yaw_rate_control()

            # publish control command
            pub_cmd = Vector3Stamped()
            pub_cmd.header.stamp = rospy.Time.now()
            pub_cmd.header.frame_id = 'auto_command_input'
            pub_cmd.vector.x = float(throttle_cmd)
            pub_cmd.vector.y = float(steering_angle)
            self.cmd_pub_.publish(pub_cmd)

            # to enforce rate/sample time
            self.rate.sleep()
            self.timestamp_last = self.timestamp

    # subscription from filternode
    def receiving(self, filter_message):
        # receive Filter results
        self.timestamp = filter_message.header.stamp
        frame_id = filter_message.header.frame_id

        self.filtered_yaw_rate = filter_message.filtered_yaw_rate
        self.filtered_counts_per_sec = filter_message.filtered_counts_per_sec
        self.filtered_longitudinal_velocity = filter_message.filtered_longitudinal_velocity
        self.filtered_lateral_velocity = filter_message.filtered_lateral_velocity
        
    # subscription from csi_lane_error_node
    def lane_error_sub(self,vision_message):
        self.lane_error = vision_message.data


def main(args):
    rospy.init_node('control_node')
    nodename = rospy.get_name()
    rospy.loginfo(f"{nodename} node started")

    r = ControlNode()

    r.run_controllers()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
