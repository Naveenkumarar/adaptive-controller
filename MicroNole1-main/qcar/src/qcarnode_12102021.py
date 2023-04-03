#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *
from qcar.q_misc import Calculus

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
import time

from odometrynode import OdometryNode
from imunode import IMUnode


class QcarNode(object):

    def __init__(self):
        super().__init__()

        # fetch ROS parameters from the server
        self.joystick_drift_tol = rospy.get_param("joystick_drift_tol", 0.002)
        self.autonomous_mode = rospy.get_param("autonomous_mode", False)  # change to a ROS topic later

        self.publish_rate = 100.0  # Hz
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
        self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)

        self.myCar = QCar()
        self.myOdometry = OdometryNode()
        self.myIMU = IMUnode()
        self.command = np.array([0, 0])
        self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)

        self.last_encoderCount = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        # miscellaneous variables

        # to initialize Quanser's differentiator
        self.diff = Calculus().differentiator_variable(self.sample_time)
        _ = next(self.diff)

    # -------------------------------------------------------------------------------------------------
    def looping(self):
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()

            # Generate Commands
            LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
            # Implement longitudinal speed control (check q_control.py).
            # Use a ROS topic to check if autonomous mode is True

            # talk to QCar
            #rospy.loginfo(f"Command: {self.command}")
            current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command,
                                                                               LEDs)  # this line is responsible for
            # sending the actuator commands to the QCar. Check "product_QCar.py" for source.
            # rospy.loginfo(f"Current = {current}, \n Encoder Counts = {encoderCounts}")

            # read IMU data
            gyroscope_readings, accelerometer_readings = self.myCar.read_IMU()

            # read battery percentage
            power, battery_percentage = power_consumption_monitor(current, batteryVoltage)

            battery_state = BatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.header.frame_id = 'battery_voltage'
            battery_state.voltage = batteryVoltage
            battery_state.percentage = battery_percentage
            # rospy.loginfo(f"Battery percentage: {battery_percentage}")
            self.battery_pub_.publish(battery_state)

            # longitudinal_car_speed = basic_speed_estimation(
            #     encoderCounts)  # current/measured speed. Check q_interpretation.py for source
            wheel_angle = self.command[1] if abs(self.command[1]) > self.joystick_drift_tol else 0.0
            # rospy.loginfo(f"Speed = {longitudinal_car_speed}, \n Wheel Angle = {wheel_angle}")
            # rospy.loginfo(f"Wheel Angle = {wheel_angle}")

            # method 1: Mine
            encoderCounts_diff = (encoderCounts - self.last_encoderCount) / self.dt  # differentiate encoder counts
            current_speed = basic_speed_estimation(encoderCounts_diff)

            # method 2: Quanser
            encoderCounts_diff2 = self.diff.send((encoderCounts, self.dt))
            current_speed2 = basic_speed_estimation(encoderCounts_diff2)
            # rospy.loginfo(f"Default Speed = {longitudinal_car_speed}, \n Adjusted Car Speed = {actual_speed}")
            # rospy.loginfo(f"Default Encoder Counts = {encoderCounts}, \n Encoder Counts diff = {encoderCounts_diff}")
            # rospy.loginfo(f"Adjusted Car Speed = {current_speed}, \t Test Car Speed = {current_speed2}")
            # rospy.loginfo(f"Difference = {current_speed - current_speed2}, \t Same = {current_speed == current_speed2}")

            longitudinal_car_speed = current_speed
            velocity_state = Vector3Stamped()
            velocity_state.header.stamp = rospy.Time.now()
            velocity_state.header.frame_id = 'car_velocity'
            # get forward component of the current speed using the kinematics
            velocity_state.vector.x = float(np.cos(self.command[1]) * longitudinal_car_speed)
            # get lateral component of the current speed using the kinematics
            velocity_state.vector.y = float(np.sin(self.command[1]) * longitudinal_car_speed)
            self.carvel_pub_.publish(velocity_state)

            # publish odometry
            self.myOdometry.publish_odom(longitudinal_car_speed, wheel_angle)

            # publish IMU data
            #rospy.loginfo(f"Gyro: {gyroscope_readings}")
            #rospy.loginfo(f"Accel: {accelerometer_readings}")
            self.myIMU.process_imu_data(gyroscope_readings, accelerometer_readings)

            # reset encoder counts
            self.last_encoderCount = encoderCounts
            self.last_time = self.current_time
            
            #if batteryVoltage <= 11.0:
                #rospy.loginfo(f"Battery voltage at {batteryVoltage}. Charge soon.")

            time.sleep(self.sample_time)

        self.myCar.terminate()

    def process_cmd(self, sub_cmd):
        rostopic.loginfo("Data received")
        vel_cmd = sub_cmd.vector.x
        #  -0.003921627998352051
        str_cmd = sub_cmd.vector.y - 0.01  # why is the -0.01 term there? Feedforward?
        #rospy.loginfo(f"Speed = {vel_cmd}, Steering angle = {str_cmd}")
        self.command = np.array([vel_cmd, str_cmd])
        
    def shutdown(self):
        # rospy.loginfo("Beginning shutdown routine...")
        self.command = np.array([0, 0])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        _, _, _ = self.myCar.read_write_std(self.command,
                                            LEDs)
        self.myCar.terminate()
        rospy.loginfo("Shutting down cleanly...")


if __name__ == '__main__':
    rospy.init_node('qcar_node')
    r = QcarNode()
    rospy.on_shutdown(r.shutdown)
    r.looping()
    rospy.spin()

