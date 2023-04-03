#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *
from qcar.q_misc import Calculus

from std_msgs.msg import String, Float32, Float64, Header
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
import time

from odometrynode import OdometryNode
from imunode import IMUnode

from filter import FilterClass

import matplotlib.pyplot as plt
import math


class QcarNode(object):

    def __init__(self, publish_odometry=True, publish_imu=True):
        super().__init__()

        # instantiate optional attributes
        self.publish_odometry = publish_odometry
        self.publish_imu = publish_imu

        # fetch ROS parameters from the server
        self.joystick_drift_tol = rospy.get_param("joystick_drift_tol", 0.002)
        self.autonomous_mode = rospy.get_param("autonomous_mode", False)  # change to a ROS topic later
        
        # subscribe lateral distance from csi_lane_error_node
        self.vision_pub = rospy.Subscriber('lane_detection/error/value', Float64, self.lane_error_sub, queue_size=100)
        self.vision_pub = rospy.Subscriber('lane_detection/error/time', Float64, self.detection_time_sub, queue_size=100)


        self.publish_rate = 200.0  # Hz
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
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.iniTime = rospy.Time.now()

        # desired states
        self.desired_wz = math.pi/12
        self.desired_speed = 0.3

        self.my_filter = FilterClass(alpha=0.95)
        self.my_filter.Theta_hat_prev = np.array([[8.79580942e+01, 8.79580942e+01],
        [2.53996905e+00, 2.53996905e+00],
        [1.29835263e+01, 1.29835263e+01],
        [3.72696260e+04, 3.72696260e+04]])
        
        self.Yaw_rate = np.array([[0]])
        self.WZ = np.array([[0]])
        self.VX = np.array([[0]])
        self.Command = np.array([[0,0]])
        self.throttle_CMD = np.array([[0]])
        self.filter_data = np.array([[0,0,0,0]])
        self.accel = np.array([[0,0]])
        self.Center_diff = np.array([[0.,0.,0.]])
        self.lane_error, self.detection_time_stamp, self.detection_time_stamp_last = 0., 0.,0.

        # controller
        self.speed_diff_last = 0.
        self.throttle_cmd_last = 0.
        
        self.desired_yaw_rate = 0.0
        
        self.Kp1 = 0.1
        self.Ki1 = 0.16
        self.integral_state1 = 0.
        
        self.Kp2 = 0.5
        self.Ki2 = 0.
        
        self.Kp3 = 1.2
        self.Ki3 = 0.5
        self.integral_state2 = 0.
        
        self.sat_pwm = 0.7
        self.rate_limit = 0.1
        
        self.steering_angle_last, self.y_last = 0., 0.
        self.tau = 0.01

        '''
        # to initialize Quanser's differentiator
        self.diff = Calculus().differentiator_variable(self.sample_time)
        _ = next(self.diff)
        '''

    # -------------------------------------------------------------------------------------------------
    def looping(self):
        while not rospy.is_shutdown():
            # odometry integration variables
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()

            self.filter_time = (self.current_time - self.iniTime).to_sec()   # time from the beginning of experiment

            # Generate Commands
            LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
            # Implement longitudinal speed control (check q_control.py).
            # Use a ROS topic to check if autonomous mode is True

            # talk to QCar
            #current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command,LEDs,sat_pwm=0.2, sat_str=0.5)  # this line is responsible for
            
            # read motor
            current, batteryVoltage, encoderCounts = self.myCar.read_std()
            # sending the actuator commands to the QCar. Check "product_QCar.py" for source.

            # read IMU data
            gyroscope_readings, accelerometer_readings = self.myCar.read_IMU()
            ax = float(accelerometer_readings[0])
            ay = float(accelerometer_readings[1])
            # az = float(accelerometer_readings[2])
            # roll_rate = float(gyroscope_readings[0])
            # pitch_rate = float(gyroscope_readings[1])
            yaw_rate = float(gyroscope_readings[2])
            
            self.Yaw_rate = np.concatenate((self.Yaw_rate,np.array([[yaw_rate]]) ),axis=0)
            
            self.accel = np.concatenate((self.accel,np.array([[ax,ay]]) ),axis=0)
            

            # read battery percentage
            power, battery_percentage = power_consumption_monitor(current, batteryVoltage)

            battery_state = BatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.header.frame_id = 'battery_voltage'
            battery_state.voltage = batteryVoltage
            battery_state.percentage = battery_percentage
            self.battery_pub_.publish(battery_state)

            wheel_angle = self.command[1] if abs(self.command[1]) > self.joystick_drift_tol else 0.0
            # rospy.loginfo(f"Speed = {longitudinal_car_speed}, \n Wheel Angle = {wheel_angle}")
            # rospy.loginfo(f"Wheel Angle = {wheel_angle}")


            # todo: incorporate controller here
            ## Observer and Conroller
            # 1. filtering

            sensor_data = np.array([self.filter_time, float(self.command[0]), float(self.command[1]), encoderCounts, yaw_rate, ax, ay])          # [time, PWM, steering, encoder counts, raw rate, ax, ay]
            # print(sensor_data.shape)
            wz_hat, wm_hat, vx_hat, vy_hat = self.my_filter.run(sensor_data)
            
            self.WZ = np.concatenate((self.WZ,np.array([[wz_hat]]) ),axis=0)
            self.VX = np.concatenate((self.VX,np.array([[vx_hat]]) ),axis=0)
            
            self.filter_data = np.concatenate((self.filter_data,np.array([[wz_hat, wm_hat, vx_hat, vy_hat]]) ),axis=0)

            # 2. PI controller (for longitudinal velocity)
            if self.detection_time_stamp-self.detection_time_stamp_last >0:
            
                self.speed_diff = self.desired_speed - vx_hat
                
                alpha = 0.01
                delta_k = 1
                self.integral_state1 = alpha * self.integral_state1 + delta_k * self.speed_diff * self.dt
                throttle_cmd = 0.1381*self.desired_speed + 3*(self.Kp1* self.speed_diff + self.Ki1 * self.integral_state1)
                
                
                if abs(throttle_cmd - self.throttle_cmd_last) > self.dt * self.rate_limit * self.sat_pwm:   # rate saturation, for safety
                    throttle_cmd = self.throttle_cmd_last +  np.sign(throttle_cmd - self.throttle_cmd_last) * self.dt * self.rate_limit * self.sat_pwm
                
                self.throttle_cmd_last = throttle_cmd

                
                self.throttle_CMD = np.concatenate((self.throttle_CMD,np.array([[throttle_cmd]])),axis=0)


                # 2. P controller (for yaw rate)
                self.yaw_rate_diff = self.desired_yaw_rate - wz_hat
                self.integral_state2 = alpha * self.integral_state2 + delta_k * self.lane_error * self.dt
                y = 0*self.Kp2 * self.yaw_rate_diff + self.Kp3 * self.lane_error + 0*self.Ki3 * self.integral_state2
                '''
                steering_angle = np.exp(-self.dt * self.tau) * self.steering_angle_last + (self.dt/2) * (y + np.exp(-self.dt*self.tau) * self.y_last)
                self.steering_angle_last = steering_angle
                self.y_last = y
                '''
                steering_angle = y
                
                steering_angle = np.sign(steering_angle) * np.minimum(np.abs(steering_angle), math.pi/6) + 0*0.025          
 
                self.command = np.array([throttle_cmd, steering_angle])
                # self.command = np.array([0., 0.])
                
                # write motors commands
                throttle_cmd_real = self.myCar.write_mtrs(self.command,sat_pwm=0.7, sat_str=0.5)
                
            self.detection_time_stamp_last = self.detection_time_stamp
            
            # self.Command = np.concatenate((self.Command,np.array([[throttle_cmd_real, steering_angle]])),axis=0)
            #self.Center_diff = np.concatenate((self.Center_diff,np.array([[self.filter_time,self.detection_time_stamp,self.lane_error]])),axis=0)


            # publish odometry
            if self.publish_odometry:
                self.myOdometry.publish_odom(vx_hat, wheel_angle)

            # publish IMU data
            '''
            if self.publish_imu:
                # todo: filter (high pass) IMU data,  integrate velocities and accelerations
                self.myIMU.process_imu_data(gyroscope_readings, accelerometer_readings, convert_units=True)

                # todo: validate QCar ROS IMU units
                # Begin validation block
                default_gravity = accelerometer_readings[-1]
                converted_imu_gyro, converted_imu_accel = self.myIMU.convert_to_ros(gyroscope_readings,
                                                                                    accelerometer_readings)
                converted_gravity = converted_imu_accel[-1]
                rospy.loginfo(f"Default gravity = {default_gravity}, Converted gravity = {converted_gravity}")

                if default_gravity >= 8.5:
                    rospy.loginfo(f"Default gravity is greater than 8.5")

                if converted_gravity >= 8.5:
                    rospy.loginfo(f"Converted gravity is greater than 8.5")
                # End validation block
                '''

            # reset encoder counts
            self.last_time = self.current_time

            # if battery_percentage <= 40.0 or batteryVoltage <= 11.0:
            if batteryVoltage <= 11.0:
                # note: Quanser's battery percentage calculation is wrong. Use voltage instead.
                rospy.loginfo(f"Battery percentage at {battery_percentage}. Battery voltage is {batteryVoltage}."
                              f" \n Charge soon.")

            time.sleep(self.sample_time)

        # self.myCar.terminate()

    # subscription from csi_lane_error_node
    def lane_error_sub(self,vision_message):
        self.lane_error = vision_message.data

    def detection_time_sub(self,meg):
        self.detection_time_stamp = meg.data

    '''
    def process_cmd(self, sub_cmd):
        vel_cmd = sub_cmd.vector.x
        str_cmd = sub_cmd.vector.y - 0.01  # why is the -0.01 term there? Feedforward?
        self.command = np.array([vel_cmd, str_cmd])
    '''
    def shutdown(self):
        # rospy.loginfo("Beginning shutdown routine...")
        self.command = np.array([0, 0])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        _, _, _ = self.myCar.read_write_std(self.command,
                                            LEDs)
        self.myCar.terminate()
        
        '''
        plt.plot(self.desired_speed * np.ones(self.VX.shape))
        plt.plot(self.VX)
        plt.show()
        
        plt.plot(self.Center_diff[1:,0],np.diff(self.Center_diff[:,1]))
        plt.title('time')
        plt.show()
        
        plt.plot(self.Center_diff[:,0], self.Center_diff[:,2])
        plt.title('center_diff')
        plt.show()
        

        plt.plot(self.Command[:,1])
        plt.show()
        
        plt.plot(self.VY)
        plt.show()
        

        plt.plot(self.accel[:,0])
        plt.plot(self.accel[:,1])
        plt.show()

        plt.plot(self.Yaw_rate,'o')
        plt.plot(self.WZ)
        plt.show()
        '''
        
        
        
        # np.savetxt(str(self.data_save_path), self.Data_save, delimiter=',', fmt='% s')
        rospy.loginfo("Shutting down cleanly...")

if __name__ == '__main__':
    rospy.init_node('qcar_node')
    r = QcarNode()
    rospy.on_shutdown(r.shutdown)

    try:
        r.looping()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        # rospy.loginfo(f'Encountered {e}.')
        # use rospy.on_shutdown() to perform interpolation and error detection
        rospy.on_shutdown(r.shutdown())
        rospy.loginfo("Shutting down")
        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)
