#!/usr/bin/env python3

"""
Publishes the following:
1. IMU [Done]
2. Encoder (kinematic odometry) [Done]
3. Current (no, for now) [Done]
4. BatteryVoltage [Done]
5. Battery Power (no) [Done]
6. Battery Percentage [Done]
7. Dr. Anubi and Yu's filter values

Subscribes to the following:
1. Steering angle [Done]
2. Velocity [Done]
3. LED (no, for now) [Done]

Control:
1. Add low-level speed controller.
2. Add steering offset for different set points.
3. (Optional) Publish Yu's XY coordinates (could use a different node).
4. Replace Autoware's control command with AckermannDriveStamped
   (or leave it as an option and use a flag/mux to decide).
5.
"""

import time

import numpy as np
import rospy

from geometry_msgs.msg import Vector3Stamped, TwistStamped, Twist
from sensor_msgs.msg import BatteryState
from autoware_msgs.msg import ControlCommand

from qcar.product_QCar import QCar
from qcar.q_interpretation import *
from qcar.q_misc import Calculus
from qcar.q_control import speed_control
from odometrynode import OdometryNode
from imunode import IMUnode


class MicronoleIONode(object):
    """docstring for ClassName"""

    def __init__(self, publish_odometry=False, publish_imu=True, publish_rate=500.0,
                 use_filter=True, publish_filter=True):
        """Constructor for MicronoleIONode"""
        super().__init__()
        # instantiate optional attributes
        self.publish_odometry = publish_odometry
        self.publish_imu = publish_imu

        self.publish_rate = publish_rate  # Hz
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
        # self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)
        self.car_actuator_pub_ = rospy.Publisher("/ctrl_cmd_actual", ControlCommand, queue_size=10)

        self.myCar = QCar()
        self.myOdometry = OdometryNode(publish_tf=True)
        self.longitudinal_velocity_pub = rospy.Publisher('/longitudinal_velocity/raw', TwistStamped, queue_size=10)
        self.myIMU = IMUnode()
        self.myIMU_debug = IMUnode(topic_name="/imu/data_raw_converted")
        self.command = np.array([0, 0])
        self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.actuator_callback, queue_size=100)
        self.ctrl_sub = rospy.Subscriber("/ctrl_cmd", ControlCommand, self.ctrl_callback,
                                         queue_size=100)
        self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.twist_callback, queue_size=10)

        # odometry variables
        self.last_encoder_count = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.dt = self.sample_time
        self.longitudinal_car_speed = 0.

        # miscellaneous variables
        self.steering_offset = 0.025 + 0.01  # +ve for left, -ve for right  # todo: read from parameter server

        # todo: setup timeout using a rospy.Timer callback to send zero commands in case no new messages are received or they are too old.

        # Filter variables
        self.use_filter = use_filter
        if self.use_filter:
            from filter import FilterClass
            self.longitudinal_filtered_velocity_pub = rospy.Publisher('/longitudinal_velocity/filtered',
                                                                      TwistStamped, queue_size=10)
            self.filtered_velocity_pub = rospy.Publisher('/velocity/filtered',
                                                         TwistStamped, queue_size=10)
            '''filter'''
            self.my_filter = FilterClass(alpha=0.95)
            self.my_filter.Theta_hat_prev = np.array([[8.79580942e+01, 8.79580942e+01],
                                                      [2.53996905e+00, 2.53996905e+00],
                                                      [1.29835263e+01, 1.29835263e+01],
                                                      [3.72696260e+04, 3.72696260e+04]])

            '''microNole model parameters'''
            self.wheelbase = 0.256  # wheelbase
            self.wheel_radius = 0.0342  # wheel radius # todo: tell Yu this should be 0.0342 or 0.066/2 instead of 0.024
            self.rg = 9.49  # total drive ratio
            self.Theta1 = 4.5355
            self.Theta2 = 12985
            self.Theta3 = 191.1749
            self.Theta4 = 85.0180
            self.Theta5 = 696.4708
            self.Theta6 = 370.3704

            ''' data cache '''
            self.filter_data = np.array([[0., 0., 0., 0., 0., 0., 0.]])  # (filter data, lane detection result)
            self.sensor_data = np.array(
                    [[0., 0., 0., 0., 0., 0.]])  # (current, voltage, encoder counts, ax, ay, yaw_rate)
            self.command_data = np.array([[0., 0.]])
            self.init_time = rospy.Time.now()
            self.filter_time = rospy.Time.now()

    def publish_data(self):
        self.last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()

            if self.use_filter:
                self.filter_time = (self.current_time - self.init_time).to_sec()

            # read motor current, battery voltage and encoder counts
            motor_current, battery_voltage, encoder_count = self.myCar.read_std()
            wheel_angle = self.command[1]  # steering angle
            encoder_speed = (encoder_count - self.last_encoder_count) / self.dt  # differentiate encoder counts
            self.longitudinal_car_speed = basic_speed_estimation(encoder_speed)

            # publish speed and steering angle as a ctrl_cmd message
            ctrl_cmd = ControlCommand()
            # ctrl_cmd.header.stamp = rospy.Time.now()
            # ctrl_cmd.header.frame_id = "actuators"
            ctrl_cmd.linear_velocity = float(self.longitudinal_car_speed)
            ctrl_cmd.steering_angle = float(wheel_angle)
            self.car_actuator_pub_.publish(ctrl_cmd)

            # publish longitudinal velocity
            self.publish_velocity(publisher=self.longitudinal_velocity_pub, frame_id='encoder',
                                  longitudinal_velocity=self.longitudinal_car_speed)

            # read battery percentage
            power, battery_percentage = power_consumption_monitor(motor_current, battery_voltage)
            battery_percentage2 = 100 - (battery_voltage - 10.5) * 100 / (12.6 - 10.5)

            # read IMU data
            gyroscope_readings, accelerometer_readings = self.myCar.read_IMU()
            ax = float(accelerometer_readings[0])
            ay = float(accelerometer_readings[1])
            az = float(accelerometer_readings[2])
            roll_rate = float(gyroscope_readings[0])
            pitch_rate = float(gyroscope_readings[1])
            yaw_rate = float(gyroscope_readings[2])

            # publish battery information
            battery_state = BatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.header.frame_id = 'battery'
            battery_state.voltage = battery_voltage
            battery_state.percentage = battery_percentage
            self.battery_pub_.publish(battery_state)

            # apply filter and publish filter data
            if self.use_filter:
                self.sensor_data = np.concatenate((self.sensor_data,
                                                   np.array([
                                                       [float(motor_current),
                                                        float(battery_voltage),
                                                        float(encoder_count),
                                                        ax, ay, yaw_rate]])), axis=0)
                '''Filter'''
                sensor_data = np.array(
                        [self.filter_time, float(self.command[0]), float(self.command[1]), encoder_count, yaw_rate, ax,
                         ay])  # [time, PWM, steering, encoder counts, raw rate, ax, ay]

                wz_hat, wm_hat, vx_hat, vy_hat = self.my_filter.run(sensor_data)
                #rospy.loginfo(f"wz_hat: {wz_hat}, wm_hat: {wm_hat}, vx_hat: {vx_hat}, vy_hat: {vy_hat}")

                # publish filtered longitudinal velocity (from filtered motor speed)
                # wheel velocity = wheel rotation(rad/s) * wheel radius(m)
                filtered_wheel_velocity = wm_hat * self.wheel_radius
                self.publish_velocity(self.longitudinal_filtered_velocity_pub, frame_id='encoder_filtered',
                                      longitudinal_velocity=filtered_wheel_velocity)

                # publish filtered velocities
                self.publish_velocity(publisher=self.filtered_velocity_pub, frame_id='base_link',
                                      longitudinal_velocity=vx_hat, lateral_velocity=vy_hat, yaw_rate=wz_hat)

            # publish odometry
            if self.publish_odometry:
                self.myOdometry.publish_odom(self.longitudinal_car_speed, wheel_angle)

            # publish raw imu data
            if self.publish_imu:
                # without converting units to ROS
                self.myIMU.process_imu_data(gyroscope_readings, accelerometer_readings, convert_units=False)

                # converting units to ROS
                self.myIMU_debug.process_imu_data(gyroscope_readings, accelerometer_readings, convert_units=True)

            self.last_time = self.current_time
            self.last_encoder_count = encoder_count
            self.rate.sleep()

    def actuator_callback(self, data):
        vel_cmd = data.vector.x
        str_cmd = data.vector.y - 0.01 + self.steering_offset  # why is the -0.01 term there? Feedforward?
        self.command = np.array([vel_cmd, str_cmd])
        desired_pwm_duty_cycle = speed_control(self.command[0], self.longitudinal_car_speed, 1, self.dt)
        self.command[0] = desired_pwm_duty_cycle
        rospy.loginfo(f"Des speed: {linear_velocity}, Current speed: {self.longitudinal_car_speed}, Desired PWM: {desired_pwm_duty_cycle}")

        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        self.myCar.write_std(self.command, LEDs)

    def ctrl_callback(self, data):
        linear_velocity = data.linear_velocity  # currently is pwm
        #linear_velocity = 2.3  # m/s
        # todo: convert desired velocity to pwm mapping
        steering_angle = data.steering_angle - 0.01 + self.steering_offset
        self.command = np.array([linear_velocity, steering_angle])

        # rospy.loginfo(f'Ctrl: Linear velocity = {linear_velocity}, Steering angle = {steering_angle}')
        # self.process_command([linear_velocity, steering_angle])

        # use low-level speed controller here to convert m/s to pwm duty cycle (see q_control.py for info)
        # speed_control(desired_speed, measured_speed, arm, dt, k_ff=0.1, k_p=0.1, k_i=0.1)
        # todo: test by measuring in the lab (set constand desired speed and plot measured speed). '/longitudinal_velocity/raw'
        desired_pwm_duty_cycle = speed_control(self.command[0], self.longitudinal_car_speed, 1, self.dt)
        self.command[0] = desired_pwm_duty_cycle

        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        #rospy.loginfo(f"Des speed: {linear_velocity}, Current speed: {self.longitudinal_car_speed}, Desired PWM: {desired_pwm_duty_cycle}")
        self.myCar.write_std(self.command, LEDs)

    def twist_callback(self, data):
        # https://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
        self.command = np.array([data.linear.x, data.angular.z])
        desired_pwm_duty_cycle = speed_control(self.command[0], self.longitudinal_car_speed, 1, self.dt)
        # todo: define method to do this to avoid redundancy
        self.command[0] = desired_pwm_duty_cycle

        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        rospy.loginfo(f"Des speed: {data.linear.x}, Current speed: {self.longitudinal_car_speed}, Desired PWM: {desired_pwm_duty_cycle}")
        self.myCar.write_std(self.command, LEDs)

    def publish_velocity(self, publisher, frame_id, longitudinal_velocity, lateral_velocity=0., yaw_rate=0.):
        longitudinal_velocity_cmd = TwistStamped()
        longitudinal_velocity_cmd.header.stamp = rospy.Time.now()
        longitudinal_velocity_cmd.header.frame_id = frame_id
        longitudinal_velocity_cmd.twist.linear.x = float(longitudinal_velocity)
        longitudinal_velocity_cmd.twist.linear.y = float(lateral_velocity)
        longitudinal_velocity_cmd.twist.angular.z = float(yaw_rate)
        publisher.publish(longitudinal_velocity_cmd)

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
        self.command = np.array([0, 0])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        _, _, _ = self.myCar.read_write_std(self.command,
                                            LEDs)
        self.myCar.terminate()
        rospy.loginfo("Shutting down cleanly...")


if __name__ == '__main__':
    rospy.init_node('qcar_node')
    mion = MicronoleIONode()
    # use rospy.on_shutdown() to perform interpolation and error detection
    rospy.on_shutdown(mion.shutdown)

    try:
        mion.publish_data()
        # rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')
        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)

