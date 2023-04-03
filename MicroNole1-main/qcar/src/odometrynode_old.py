#!/usr/bin/env python3

"""
To calculate the odometry of the car.
"""

# import modules
from __future__ import division, print_function, absolute_import

import math
from math import sin, cos, tan, pi
import sys
import rospy
import tf
import numpy as np

# import ros messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped

# import QCar modules

# import utilities


class OdometryNode(object):
    """docstring for ClassName"""

    def __init__(self, wheelbase=0.256):
        """Constructor for OdometryNode"""
        super().__init__()

        # assign parameters to default values of rosparam variables
        self.WHEELBASE = rospy.get_param('wheelbase', wheelbase)

        # queue sizes

        # setup ros subscribers
        # todo: remove when merged with QCar node
        # self.command = np.array([0, 0])
        # self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)

        # setup ros publishers
        self.odom_put = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # initialize variables
        self.publish_rate = 500.0  # Hz
        self.rate = rospy.Rate(self.publish_rate)

        # odometry variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0  # m/s (vx = speed)
        self.vy = 0.0  # m/s (vy = 0)
        self.vth = 0.0  # rad/s (vth = ((right_speed - left_speed)/lengthWheelBase))

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def looping(self):
        while not rospy.is_shutdown():
            speed = self.vx
            wheel_angle = self.th
            self.publish_odom(speed=speed, wheel_angle=wheel_angle)

    def publish_odom(self, speed, wheel_angle):
        # rospy.loginfo("Entered 'publish_odom' function.")
        self.current_time = rospy.Time.now()

        # todo: remove when merged with QCar node
        # current, batteryVoltage, encoderCounts = self.myCar.read_std()

        self.vx = speed  # m/s (vx = speed)
        self.vy = 0  # m/s (vy = 0)
        self.vth = self.vx * tan(wheel_angle) / self.WHEELBASE  # rad/s

        # compute odometry in a typical way given the velocities of the robot
        # todo: check Quanser's initial code for kinematics calculation
        # todo: 
        self.dt = (self.current_time - self.last_time).to_sec()

        # test1
        self.delta_x_1 = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        self.delta_y_1 = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        # rospy.loginfo(f"dx1: {self.delta_x_1}, dy1: {self.delta_y_1}")

        # test2
        self.delta_x_2 = (self.vx * cos(wheel_angle) - self.vy * sin(wheel_angle)) * self.dt
        self.delta_y_2 = (self.vx * sin(wheel_angle) + self.vy * cos(wheel_angle)) * self.dt
        # rospy.loginfo(f"dx2: {self.delta_x_1}, dy2: {self.delta_y_1}")
        
        # test3
        self.delta_x_3 = self.vx * cos(self.th)
        self.delta_y_3 = self.vx * sin(self.th)
        # rospy.loginfo(f"dx3: {self.delta_x_1}, dy3: {self.delta_y_1}")

        self.delta_x = self.delta_x_1
        self.delta_y = self.delta_y_1
        self.delta_th = self.vth * self.dt

        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                self.odom_quat,
                self.current_time,
                "base",
                "odom"
        )

        # next, we'll publish the odometry message over ROS
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"

        # set the position
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

        # set the velocity
        self.odom.child_frame_id = "base"
        self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_put.publish(self.odom)

        self.last_time = self.current_time

        # to enforce rate/sample time
        # rospy.loginfo("Leaving 'publish_odom' function.")
        # self.rate.sleep()

    def process_cmd(self, sub_cmd):
        vel_cmd = sub_cmd.vector.x
        str_cmd = sub_cmd.vector.y - 0.01  # why is the -0.01 term there? Feedforward?
        self.command = np.array([vel_cmd, str_cmd])

    def process_stuff(self):
        pass

    def publish(self):
        pass


def main(args):
    rospy.init_node('odometry_publisher')
    nodename = rospy.get_name()
    rospy.loginfo(f"{nodename} node started")
    odon = OdometryNode()
    odon.looping()

    try:
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        print(f'Encountered {e}. Shutting down.')

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    main(sys.argv)
