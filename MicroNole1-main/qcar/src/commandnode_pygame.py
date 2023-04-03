#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.q_misc import *
from qcar.q_ui import *
from qcar.q_interpretation import *

from geometry_msgs.msg import Vector3Stamped
import time
import pygame

pygame.init()
pygame.display.set_mode([100, 100])


class CommandNode(object):
    def __init__(self):
        super().__init__()
        # self.gpad = gamepadViaTarget(5)
        self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)
        throttle_cmd = np.array([0, 0])
        
        # to change the max while running (dynamically)
        linear_velocity = 0.1
        angular_velocity = 0.1
        
        # actuator limits
        max_linear_velocity = 0.3
        max_angular_velocity = 0.5

        while not rospy.is_shutdown():

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

                if event.type == pygame.KEYDOWN:
                    # move the QCar
                    if event.key == pygame.K_w:
                        if throttle_cmd[0] != (0.3 - 0.2):
                            throttle_cmd = throttle_cmd + np.array([(0.3 - 0.2), 0])
                    if event.key == pygame.K_s:
                        if throttle_cmd[0] != -(0.3 - 0.2):
                            throttle_cmd = throttle_cmd + np.array([-(0.3 - 0.2), 0])
                    if event.key == pygame.K_a:
                        if throttle_cmd[1] != (0.5 - 0.25):
                            throttle_cmd = throttle_cmd + np.array([0, (0.5 - 0.25)])
                    if event.key == pygame.K_d:
                        if throttle_cmd[1] != -(0.5 - 0.25):
                            throttle_cmd = throttle_cmd + np.array([0, -(0.5 - 0.25)])
            self.process_command(throttle_cmd)
            time.sleep(0.01)
        # self.gpad.terminate()

    # --------------------------------------------------------------------------------------------

    def process_command(self, pose):
        pub_cmd = Vector3Stamped()
        pub_cmd.header.stamp = rospy.Time.now()
        pub_cmd.header.frame_id = 'command_input'
        pub_cmd.vector.x = float(pose[0])  # speed
        pub_cmd.vector.y = float(pose[1])  # steering angles
        self.cmd_pub_.publish(pub_cmd)


if __name__ == '__main__':
    rospy.init_node('command_node')
    r = CommandNode()

    rospy.spin()
