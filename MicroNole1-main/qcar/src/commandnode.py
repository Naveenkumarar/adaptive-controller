#!/usr/bin/env python3

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import time
import sys

# third-party library imports
import numpy as np
import roslib
import rospy
from geometry_msgs.msg import Vector3Stamped

# import local modules
from qcar.q_misc import *
from qcar.q_ui import *
from qcar.q_interpretation import *


class CommandNode(object):
    def __init__(self, publish_rate=500.0):
        super().__init__()

        # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
        rospy.on_shutdown(self.shutdown)

        # run "ls -l /dev/input/by-id" and add 1 to the event number
        self.joystick_event_no = rospy.get_param("~joystick_event_no", 5)
        self.gpad = gamepadViaTarget(self.joystick_event_no)
        # todo: change topic name to /micronole/joystick_command
        self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = publish_rate  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

    def read_commands(self):
        while not rospy.is_shutdown():
            # left_lateral, left_longitudinal, right_lateral, right_longitudinal, LT, RT, A, B, X, Y, LB, RB, BACK, START, Logitech, hat = gamepad_io_qcar() # .................... Logitech......................
            new = self.gpad.read()
            pose = control_from_gamepad(self.gpad.LB, self.gpad.RT, self.gpad.LLA, self.gpad.A)
            self.process_command(pose, new)
            # to enforce rate/sample time
            self.rate.sleep()
            # time.sleep(0.01)
        # self.gpad.terminate()

    # --------------------------------------------------------------------------------------------

    def process_command(self, pose, new):
        if new:
            pub_cmd = Vector3Stamped()
            pub_cmd.header.stamp = rospy.Time.now()
            pub_cmd.header.frame_id = 'command_input'
            pub_cmd.vector.x = float(pose[0])
            pub_cmd.vector.y = float(pose[1])
            self.cmd_pub_.publish(pub_cmd)

    def shutdown(self):
        rospy.loginfo("Beginning clean shutdown routine...")
        # perform shutdown tasks here
        pose = np.array([0., 0.])
        self.process_command(pose, new=True)
        self.gpad.terminate()
        rospy.loginfo("Shutting down...")


def main(args):
    optional_nodename = 'command_node'  # this will be overwritten if a nodename is specified in a launch file
    rospy.init_node(f'{optional_nodename}')
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo(f"{nodename} node started.")  # just log that the node has started.
    controller_instance = CommandNode(publish_rate=500.0)  # 135 Hz is the max the joystick can do

    try:
        # run the main functions here
        controller_instance.read_commands()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        # this exception block most likely won't be called since there is a shutdown method in the class that will
        # override this and shutdown however is needed but is here just in case.
        rospy.loginfo(f'Encountered {e}. Shutting down.')

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    main(sys.argv)
