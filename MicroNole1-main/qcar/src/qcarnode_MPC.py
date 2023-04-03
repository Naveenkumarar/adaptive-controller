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
        self.vision_pub = rospy.Subscriber('lane_detection/error/value', Float64, self.lane_error_sub, queue_size=1)
        self.vision_pub2 = rospy.Subscriber('lane_detection/error/time', Float64, self.detection_time_sub, queue_size=1)
        self.vision_pub3 = rospy.Subscriber('lane_detection/error/laneCnenter', Float64, self.lane_center_sub, queue_size=1)

        # frequency setting
        self.publish_rate = 1000.0  # Hz (max:3448 Hz)
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        # publisher
        self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
        self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)

        self.myCar = QCar()
        self.myOdometry = OdometryNode()
        self.myIMU = IMUnode()
        self.command = np.array([0, 0])
        self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)

        # time variables
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.iniTime = rospy.Time.now()

        '''filter'''
        self.my_filter = FilterClass(alpha=0.95)
        self.my_filter.Theta_hat_prev = np.array([[8.79580942e+01, 8.79580942e+01],
        [2.53996905e+00, 2.53996905e+00],
        [1.29835263e+01, 1.29835263e+01],
        [3.72696260e+04, 3.72696260e+04]])

        # pose estimation
        self.x, self.y, self.Phi = 0., 0., 0.
        
        '''microNole model parameters (from model characterization)'''
        self.L = 0.256   # wheelbase
        self.Rw = 0.024  # wheel radius
        self.rg = 9.49   # total drive ratio
        self.Theta1 = 4.5355
        self.Theta2 = 12985
        self.Theta3 = 191.1749
        self.Theta4 = 85.0180
        self.Theta5 = 696.4708
        self.Theta6 = 370.3704
        

        '''controllers'''
        self.e1, self.e1_dot, self.e2, self.e2_dot = 0., 0., 0., 0.
        self.e1_last, self.e2_last = 0., 0.
        self.dt = 0.0

        self.throttle_cmd_last, self.steering_cmd_last = 0., 0.
        self.speed_diff = 0.
        self.curvem = 1e+5
        
        # 1. throttle controller (PI)
        self.desired_longitudinal_speed = 0.7      # desired longitudinal speed
        self.Kp1 = 0.1
        self.Ki1 = 0.48
        self.integral_state1 = 0.
        self.K1 = self.Theta1*2*self.Theta5/(self.Theta2*(2*self.Theta5*self.Rw/self.rg)) # feedforward gain
        
        
        # 2. lane following control (MPC)
        self.d0 = 0.
        self.lane_error, self.detection_time_stamp, self.detection_time_stamp_last = self.d0, 0.,0.   # for lane detection
        
        self.K_mpc_counterclock = np.array([[-0.0016, 0.2954, -0.6669, -0.3323]])  # control gains are solved based on MPC
        self.K_mpc_clock = np.array([[0.0016, 0.3940, -0.7122, -0.3169]])         # please go to matlab simulation to check how we get the gains
        
        self.sat_pwm = 0.7
        self.sat_steering = math.pi/6
        self.rate_limit = 0.1   # for throttle cmd
        self.rate_limit2 = 0.05
        
        self.clockwise_flag = 1
        
        # self.steering_angle_last, self.y_last = 0., 0.
        # self.tau = 0.01
        ''' data cache '''
        self.filter_data = np.array([[0.,0.,0.,0.,0.,0.,0.]])  # (filter data, lane detection result)
        self.sensor_data = np.array([[0.,0.,0.,0.,0.,0.]]) # (current, voltage, encoder counts, ax, ay, yaw_rate)
        self.command_data = np.array([[0.,0.]]) 
        self.steering = 0.

    # -------------------------------------------------------------------------------------------------
    def looping(self):
        while not rospy.is_shutdown():
            # odometry integration variables
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()          # sampling time for filter and controller
            self.filter_time = (self.current_time - self.iniTime).to_sec()   # time from the beginning of experiment
            # rospy.loginfo(f"loop time gap:{self.dt}")
            

            '''Read Sensors'''
            # read motor
            current, batteryVoltage, encoderCounts = self.myCar.read_std()

            # read IMU data
            gyroscope_readings, accelerometer_readings = self.myCar.read_IMU()
            ax = float(accelerometer_readings[0])
            ay = float(accelerometer_readings[1])
            # az = float(accelerometer_readings[2])
            # roll_rate = float(gyroscope_readings[0])
            # pitch_rate = float(gyroscope_readings[1])
            yaw_rate = float(gyroscope_readings[2])

            # read battery percentage
            power, battery_percentage = power_consumption_monitor(current, batteryVoltage)
            self.publisher_power(batteryVoltage,battery_percentage)

            # save sensor data
            self.sensor_data = np.concatenate((self.sensor_data,np.array([[float(current), float(batteryVoltage), float(encoderCounts), ax, ay, yaw_rate]]) ),axis=0)


            '''Filter'''
            # 1. velocities
            sensor_data = np.array([self.filter_time, float(self.command[0]), float(self.command[1]), encoderCounts, yaw_rate, ax, ay])     # [time, PWM, steering, encoder counts, raw rate, ax, ay]
            wz_hat, wm_hat, vx_hat, vy_hat = self.my_filter.run(sensor_data)
            
            self.filter_data = np.concatenate((self.filter_data,np.array([[self.filter_time, wz_hat, wm_hat, vx_hat, vy_hat, self.detection_time_stamp, self.lane_error]]) ),axis=0)

            # 2. pose estimation based on forward kinematics
            self.x, self.y, self.Phi = self.pose_estimation(vx_hat,vy_hat,self.command[1])

            '''Controller'''
            # error interpolation
            detction_time_inloop = self.detection_time_stamp

            if detction_time_inloop-self.detection_time_stamp_last >0:   # consider detection delay
                self.e1 = -1.0 * self.lane_error
                self.e2 = self.Phi - math.atan(1/self.curvem)
            else:
                self.e1, self.e2 = self.lane_error_interpolation(self.e1,self.e2,wz_hat)
            
            # prerpare error vector
            # [center offset, first derivative of center offset, yaw error, first derivative of yaw error]
            self.e1_dot = (self.e1 - self.e1_last) / self.dt
            self.e1_dot = (self.e1 - self.e1_last) / self.dt 
            e = np.array([[self.e1], [self.e1_dot], [self.e2], [self.e2_dot]])

            # lateral control
            steering_cmd = self.lane_following_control(e,vx_hat,wm_hat)

            # longitudinal control
            throttle_cmd = self.velocity_control(vx_hat)

            # reset cache for errors
            self.e1_last = self.e1
            self.e2_last = self.e2


            '''send command'''
            self.command = np.array([throttle_cmd, steering_cmd])
            LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            if steering_cmd > 0.3:
                LEDs[0] = 1
                LEDs[2] = 1
            elif steering_cmd < -0.3:
                LEDs[1] = 1
                LEDs[3] = 1
            if throttle_cmd < 0:
                LEDs[5] = 1
                    
            # write motors commands
            #throttle_cmd_real = self.myCar.write_mtrs(self.command,sat_pwm=0.2, sat_str=0.5)
            current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command, LEDs)
            

            ''' save data '''
            self.command_data = np.concatenate((self.command_data,np.array([[throttle_cmd, steering_cmd]]) ),axis=0)

            # update loop time
            self.last_time = self.current_time

            # update lane detection time   
            self.detection_time_stamp_last = detction_time_inloop

            if batteryVoltage <= 11.0:
                rospy.loginfo(f"Battery percentage at {battery_percentage}. Battery voltage is {batteryVoltage}."
                              f" \n Charge soon.")

            time.sleep(self.sample_time) # set fixed frequency

    def pose_estimation(self,vx,vy,delta):
        '''
        vx: longitudinal velocity
        vy: latera velocity
        delta: steering cmd
        '''
        v = math.sqrt(vx**2 + vy**2)
        beta = math.atan(math.tan(delta)/2)

        x_next = self.x + v * math.cos(self.Phi + beta) * self.dt
        y_next = self.y + v * math.sin(self.Phi + beta) * self.dt
        Phi_next = self.Phi + self.dt * v * math.cos(beta) * math.tan(delta) / self.L

        return x_next, y_next, Phi_next
                
    def lane_following_control(self,e,vx_hat,wm_hat):
        
        # e = -self.lane_error
        # (e positive if car is at right of lane, e negative if car is at left of lane)
        
        # identify control gain
        K_mpc = self.K_mpc_counterclock
        # rospy.loginfo(f"clockwise flag: {self.clockwise_flag}")
        #if self.clockwise_flag == 1:
        #    K_mpc = self.K_mpc_clock
        
        # feedforward term
        vx = self.desired_longitudinal_speed
        b1 = self.Theta6 * vx + self.Theta5 * self.Rw * wm_hat/self.rg
        b2 = self.Theta3 * self.L * vx/2 + self.Rw * self.L * self.Theta4 * wm_hat/(2*self.rg)
        # rospy.loginfo(f"b2:{b2}")
        R = self.curvem

        ff = -K_mpc[0,2]*(self.Theta5-vx)/(2*R*self.Theta6) + self.L**2 * vx * (self.Theta3+self.Theta4)/(2*R*b2) - b1*K_mpc[0,2]*self.L**2 * (self.Theta3+self.Theta4)/(4*R*self.Theta6*b2)
        
        # control input
        return np.dot(K_mpc,e) + ff
        
    def velocity_control(self,vx_hat):
        '''longitudinal velocity control'''
        # slow down when turn
        gain = np.minimum( 1.0, np.maximum(np.cos(self.steering), 0.5) )
        vx_desired = gain * self.desired_longitudinal_speed

        # PI cotroller
        self.speed_diff = vx_desired - vx_hat  # longitudinal speed difference
                
        alpha = 0.01
        delta_k = 1
        self.integral_state1 = alpha * self.integral_state1 + delta_k * self.speed_diff * self.dt
        throttle_cmd = self.K1*vx_desired + self.Kp1* self.speed_diff + self.Ki1 * self.integral_state1    # 0.1381 is the feedforward gain from model analysis
        
        # rate saturation
        if abs(throttle_cmd - self.throttle_cmd_last) > self.dt * self.rate_limit * self.sat_pwm:   # rate saturation, for safety
            throttle_cmd = self.throttle_cmd_last +  np.sign(throttle_cmd - self.throttle_cmd_last) * self.dt * self.rate_limit * self.sat_pwm
        
        self.throttle_cmd_last = throttle_cmd
        
        return throttle_cmd
        
    def lane_error_interpolation(self,e1,e2,wz_hat):
        ''' based on bicycle kinematic at center of gravity: Vy = L*yaw_rate/2
        This function do the iterpolation using a simplified model:
        e1_{k+1} = e1_k - Vy*dt
        (e1 positive if car is at right of lane, e negative if car is at left of lane)
        e2_{k+1} = e2_k + wz_hat*dt
        '''
        e1_next = e1 - self.dt*(self.L*wz_hat)/2
        e2_next = e2 + wz_hat*self.dt

        return e1_next, e2_next

    '''Publishers'''
    def publisher_power(self,voltage,percentage):
            battery_state = BatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.header.frame_id = 'battery_voltage'
            battery_state.voltage = voltage
            battery_state.percentage = percentage
            self.battery_pub_.publish(battery_state)


    '''subscription functions'''
    #----------------------------------------------------------------------------------------
    def lane_error_sub(self,vision_message):
        self.lane_error = vision_message.data

    def detection_time_sub(self,msg):
        self.detection_time_stamp = abs(msg.data)
        if msg.data < 0:
            self.clockwise_flag = 0   # if recive time is a negative value, reverse it and run counter clockwise steering control

    def lane_center_sub(self,msg):
        self.curvem = msg.data

    def process_cmd(self, sub_cmd): 
        '''Gamepad command'''
        vel_cmd = sub_cmd.vector.x
        str_cmd = sub_cmd.vector.y - 0.01  # why is the -0.01 term there? Feedforward?
        self.command = np.array([vel_cmd, str_cmd])
    #----------------------------------------------------------------------------------------

    def shutdown(self):
        # rospy.loginfo("Beginning shutdown routine...")
        self.command = np.array([0, 0])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        _, _, _ = self.myCar.read_write_std(self.command,
                                            LEDs)
        self.myCar.terminate()

        # save experiment data
        np.savetxt('sensor_data.csv', self.sensor_data, delimiter=',', fmt='% s')
        np.savetxt('filter_data.csv', self.filter_data, delimiter=',', fmt='% s')
        np.savetxt('commands.csv', self.command_data, delimiter=',', fmt='% s')
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

