#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import String
import time

from qcar.product_QCar import QCar


import tf  # Causes errors. Might have to just use the "pyquarternion" package


class IMUnode(object):
    def __init__(self, topic_name='/imu/data_raw'):
        super().__init__()
        # IMU provides data at a rate of 500 Hz (Check page 6 of "IV Software - Simulink.pdf" for reference
        self.imu_queue_size = 1
        self.imu_raw_pub = rospy.Publisher(f'{topic_name}', Imu, queue_size=self.imu_queue_size)
        # self.imu_filtered_pub = rospy.Publisher('/imu_filtered', Imu, queue_size=self.imu_queue_size)
        self.publish_rate = 500  # Hz
        self.sample_time = 1 / self.publish_rate
        self.arm = 1  # send to the parameter server
        rate = rospy.Rate(self.publish_rate)  # 500hz

    def looping(self):
        while not rospy.is_shutdown():
            # gyroscope_readings, accelerometer_readings = self.myCar.read_IMU()
            # publish the data here
            # time.sleep(self.sample_time)
            # rate.sleep()
            pass

        # Get IMU data.
        # Filter data.
        # Publish the IMU data.

    def filter_imu_data(self, something):
        # to filter the IMU data
        pass

    def process_imu_data(self, gyroscope_readings, accelerometer_readings, convert_units=True):
        """
        Check https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html for details
        """
        # convert default units to ROS compatible units
        if convert_units:
            gyroscope_readings, accelerometer_readings = self.convert_to_ros(gyroscope_readings, accelerometer_readings)

        # to publish the IMU data
        imu_raw_cmd = Imu()
        imu_raw_cmd.header.stamp = rospy.Time.now()
        imu_raw_cmd.header.frame_id = 'imu_link'

        # process the gyroscope data (should be in rads/sec)
        imu_raw_cmd.angular_velocity.x = float(gyroscope_readings[0])
        imu_raw_cmd.angular_velocity.y = float(gyroscope_readings[1])
        imu_raw_cmd.angular_velocity.z = float(gyroscope_readings[2])
        imu_raw_cmd.angular_velocity_covariance = [0.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0]

        # process the accelerometer data (should be in m/s^2)
        imu_raw_cmd.linear_acceleration.x = float(accelerometer_readings[0])
        imu_raw_cmd.linear_acceleration.y = float(accelerometer_readings[1])
        imu_raw_cmd.linear_acceleration.z = float(accelerometer_readings[2])
        imu_raw_cmd.linear_acceleration_covariance = [0.0, 0.0, 0.0,
                                                      0.0, 0.0, 0.0,
                                                      0.0, 0.0, 0.0]

        # process orientation data
        # roll, pitch, yaw = gyroscope_readings  # wrong. RHS must be integrated to get LHS
        # # Method1: use the package to filter the imu data and get orientation (better)
        # # Method2
        # q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # imu_raw_cmd.orientation.x = q[0]
        # imu_raw_cmd.orientation.y = q[1]
        # imu_raw_cmd.orientation.z = q[2]
        # imu_raw_cmd.orientation.w = q[3]
        # imu_raw_cmd.orientation_covariance = [-1, 0.0, 0.0,
        #                                       0.0, 0.0, 0.0,
        #                                       0.0, 0.0, 0.0]

        # publish the data
        self.imu_raw_pub.publish(imu_raw_cmd)

    def convert_to_ros(self, gyroscope_readings, accelerometer_readings):
        # convert from g's to m/s^2
        gyroscope_readings = [axis * 0.0174532925 for axis in gyroscope_readings]

        # convert from deg/s to rad/s
        accelerometer_readings = [axis * 9.80665 for axis in accelerometer_readings]
        return gyroscope_readings, accelerometer_readings


if __name__ == '__main__':
    rospy.init_node('imu_node')
    r = IMUnode()

    rospy.spin()
