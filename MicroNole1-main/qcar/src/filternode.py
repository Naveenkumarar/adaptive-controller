#!/usr/bin/env python3

''' Implement filters to fuse imu, encoder data in order to obtain MicroNole's longitudinal velocity and yaw rate
Inputs:
        - float ax                 # accelometer longitudinal reading
        - float ay                 # accelometer lateral reading
        - float az                 # accelometer updown reading
        - float roll_rate          # gyroscope roll rate
        - float pitch_rate         # gyroscope pitch rate
        - float yaw_rate           # gyroscope yaw rate
        - float encoderCounts      # Motor encoder counts
        - float current            # motor current
        - float batteryVoltage     # battery voltage
        - float filter_time        # absoluate experiment time (experiement starts at 0s)
        - float throttle_cmd       # Motor throttle PWM(%) command
        - float steering_angle     # Steering command
        
'''

from __future__ import division, print_function, absolute_import
import sys

import numpy as np
import roslib
import rospy

from filter import FilterClass
from qcar.msg import FilterData, SensorData


class FilterNode(object):
    def __init__(self, publish_filter=True):
        super().__init__()

        # subscribe sensor readings from qcarnode
        self.sensor_hub = rospy.Subscriber('/qcar/sensorReadings', SensorData, self.sensor_readings, queue_size=100)

        # initialize publisher
        self.filter_hub = rospy.Publisher('/qcar/filter_data', FilterData, queue_size=100)
        self.publish_filter = publish_filter

        # initialize filter
        self.my_filter = FilterClass(alpha=0.95)
        # This parameter initialization is only for MicroNole, otherwise set to zero matrix
        self.my_filter.Theta_hat_prev = np.array([[8.79580942e+01, 8.79580942e+01],
        [2.53996905e+00, 2.53996905e+00],
        [1.29835263e+01, 1.29835263e+01],
        [3.72696260e+04, 3.72696260e+04]])

        # data storage cache
        self.filter_data = np.array([[0, 0, 0, 0]])

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = 100.0  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)
        
        # initialize data
        self.filter_time, self.throttle_cmd, self.steering_angle, self.encoderCounts, self.yaw_rate, self.ax, self.ay = 0., 0., 0., 0., 0., 0., 0.
    
    def run_filter(self):
        while not rospy.is_shutdown():
            # run filter
            filter_sensor_data = np.array([self.filter_time, self.throttle_cmd, self.steering_angle, self.encoderCounts, self.yaw_rate, self.ax, self.ay])          # [time, PWM, steering, encoder counts, raw rate, ax, ay]
            wz_hat, wm_hat, vx_hat, vy_hat = self.my_filter.run(filter_sensor_data)

            # store data
            self.filter_data = np.concatenate((self.filter_data, np.array([[wz_hat, wm_hat, vx_hat, vy_hat]]) ),axis=0)

            # publish sensor data
            if self.publish_filter:
                filter_results = FilterData()
                filter_results.header.stamp = rospy.Time.now()
                filter_results.header.frame_id = 'filter_results'
                filter_results.filtered_yaw_rate = wz_hat
                # To do: change the message later
                filter_results.filtered_counts_per_sec = wm_hat
                filter_results.filtered_longitudinal_velocity = vx_hat
                filter_results.filtered_lateral_velocity = vy_hat
                # filter_results.filter_time = self.filter_time

                self.filter_hub.publish(filter_results)

            # rospy.loginfo(f"{wz_hat}, {wm_hat}, {vx_hat}, {vy_hat}")
            # to enforce rate/sample time
            self.rate.sleep()

    # subscription from qcarnode
    def sensor_readings(self, sensor_message):
        timestamp = sensor_message.header.stamp
        frame_id = sensor_message.header.frame_id
        # IMU 
        self.ax = sensor_message.ax
        self.ay = sensor_message.ay
        self.az = sensor_message.az
        self.roll_rate = sensor_message.roll_rate
        self.pitch_rate = sensor_message.pitch_rate
        self.yaw_rate = sensor_message.yaw_rate
        
        # rospy.loginfo(f"Yaw_rate:{self.yaw_rate}")
        
        # encoder
        self.encoderCounts =  sensor_message.encoderCounts
        
        # power
        self.current = sensor_message.current
        self.batteryVoltage = sensor_message.batteryVoltage
        
        # time
        self.filter_time = sensor_message.filter_time
        
        # commands
        self.throttle_cmd = sensor_message.throttle_cmd
        self.steering_angle = sensor_message.steering_angle


def main(args):
    rospy.init_node('filter_node')
    nodename = rospy.get_name()
    rospy.loginfo(f"{nodename} node started")

    r = FilterNode()

    r.run_filter()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
