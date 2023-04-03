#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import math
import roslib
import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *
from qcar.q_misc import Calculus

from std_msgs.msg import String, Float32
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
import time

from odometrynode import OdometryNode
from imunode import IMUnode


# Takes center

# Does calculation

# Sends commands to QCar

class QcarNode(object):

    def __init__(self, publish_odometry=False, publish_imu=False):
        super().__init__()

        # instantiate optional attributes
        self.error = 0.
        self.publish_odometry = publish_odometry
        self.publish_imu = publish_imu

        # fetch ROS parameters from the server
        self.joystick_drift_tol = rospy.get_param("joystick_drift_tol", 0.002)
        self.autonomous_mode = rospy.get_param("autonomous_mode", False)  # change to a ROS topic later

        self.publish_rate = 500 # Hz
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
        self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)

        self.myCar = QCar()
        self.myOdometry = OdometryNode()
        self.myIMU = IMUnode()
        self.command = np.array([0, 0])
        # self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)

        # odometry variables
        self.last_encoderCount = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        # miscellaneous variables
        self.count = 0
        self.gidx = 0
        self.error_last = 0
        self.ts = 0.1

        # to initialize the differentiator
        self.diff = Calculus().differentiator_variable(self.sample_time)
        _ = next(self.diff)

        # subscribe to Error message topics
        queue_size = 1
        self.error_sub = rospy.Subscriber("lane_detection/error/value", Float64, self.error_callback,
                                          queue_size=queue_size)
        self.error_header_sub = rospy.Subscriber("lane_detection/error/time", Header, self.error_header_callback,
                                                 queue_size=queue_size)

    # -------------------------------------------------------------------------------------------------
    def looping(self):
        while not rospy.is_shutdown():
            # odometry integration variables
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()

            # Generate Commands
            LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

            # Call Ashwins function
            self.interpret_error()

            # talk to QCar
            current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command,
                                                                               LEDs)  # this line is responsible for
            # sending the actuator commands to the QCar. Check "product_QCar.py" for source.

            # read battery percentage
            power, battery_percentage = power_consumption_monitor(current, batteryVoltage)

            battery_state = BatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.header.frame_id = 'battery_voltage'
            battery_state.voltage = batteryVoltage
            battery_state.percentage = battery_percentage
            self.battery_pub_.publish(battery_state)

            # longitudinal_car_speed = basic_speed_estimation(
            #     encoderCounts)  # current/measured speed. Check q_interpretation.py for source
            wheel_angle = self.command[1] if abs(self.command[1]) > self.joystick_drift_tol else 0.0
            # rospy.loginfo(f"Speed = {longitudinal_car_speed}, \n Wheel Angle = {wheel_angle}")
            # rospy.loginfo(f"Wheel Angle = {wheel_angle}")

            # method 1: differentiate encoderCounts (distance) to approximate velocity
            encoderCounts_diff = (encoderCounts - self.last_encoderCount) / self.dt  # differentiate encoder counts
            current_speed = basic_speed_estimation(encoderCounts_diff)

            longitudinal_car_speed = current_speed
            velocity_state = Vector3Stamped()
            velocity_state.header.stamp = rospy.Time.now()
            velocity_state.header.frame_id = 'car_velocity'
            # get forward component of the current speed using the kinematics
            velocity_state.vector.x = float(np.cos(self.command[1]) * longitudinal_car_speed)
            # get lateral component of the current speed using the kinematics
            velocity_state.vector.y = float(np.sin(self.command[1]) * longitudinal_car_speed)
            self.carvel_pub_.publish(velocity_state)

            # reset encoder counts
            self.last_encoderCount = encoderCounts
            self.last_time = self.current_time

            time.sleep(self.sample_time)
            #self.rate.sleep()
        self.myCar.terminate()

    def process_cmd(self, speed, steering_angle):
        # vel_cmd = sub_cmd.vector.x
        # str_cmd = sub_cmd.vector.y - 0.01
        self.command = np.array([speed, steering_angle])

    def error_callback(self, data):
        self.error = data.data
        rospy.loginfo(f"Error = {self.error}")

    def error_header_callback(self, data):
        self.error_time_stamp = data.stamp
        self.error_frame_id = data.frame_id

    def interpret_error(self):
        self.side_pos = 'left'
        if self.error <= 0:
            self.side_pos = 'right'

        Kp, Kd = 1.2, 0
        y = Kp * self.error + Kd * ((self.error - self.error_last) / self.ts)
        steering_angle = -np.sign(y) * np.minimum(np.abs(y), math.pi / 6)

        # store previous error
        self.error_last = self.error
        rospy.loginfo(f"Steering angle suggested: {steering_angle}")
        self.process_cmd(speed=0.07, steering_angle=steering_angle)
    
    def shutdown(self):
        self.process_cmd(speed=0., steering_angle=0.)
        self.myCar.terminate()


if __name__ == '__main__':
    rospy.init_node('qcar_node')
    r = QcarNode()   
    
    try:
        r.looping()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')
        # use rospy.on_shutdown() to perform interpolation and error detection
        rospy.on_shutdown(r.shutdown())
        rospy.loginfo("Shutting down")
        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)