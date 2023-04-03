#!/usr/bin/env python3

"""
The purpose of this node is to monitor topics (e.g LaserScan) and shut down the motors once terminated.

"""

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import sys
import time
import struct

# third-party library imports
import rospy

# import ros messages and services
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Empty

# import local modules
from qcar.q_essential import LIDAR
from qcar.product_QCar import QCar


class HardwareWatchDogNode(object):
    """docstring for ClassName"""

    def __init__(self, subscriber1_topic='/scan', publisher1_topic='/sample/publisher'):
        """Constructor for OdometryNode"""
        super().__init__()

        # setup queue sizes
        publisher1_queue_size = 10
        subscriber1_queue_size = 10

        # # setup ros subscribers
        # self.subscriber1_object = rospy.Subscriber(f'{subscriber1_topic}', Vector3Stamped,
        #                                            self.sample_callback1, queue_size=subscriber1_queue_size)

        # setup ros publishers
        self.publisher1_object = rospy.Publisher(f"{publisher1_topic}", Vector3Stamped,
                                                 queue_size=publisher1_queue_size)

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = 10.0  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

        # initialize other variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # to get the current ROS time. Use this function call as it works both for simulation and wall time.
        self.current_time = rospy.Time.now()
        current_time_seconds = self.current_time.to_sec()  # to convert a ROS time object to seconds.

        self.lidar_motor_stop_service = "stop_motor"
        rospy.wait_for_service(self.lidar_motor_stop_service)
        rospy.loginfo(f"Finished waiting for service.")

    def run(self):
        while not rospy.is_shutdown():
            # process loop here. Put in whatever functions and calculations need to be run here.
            # publish data after calculation if necessary
            self.sample_publisher(1)

            # to enforce rate/sample time
            self.rate.sleep()

        # whatever is written here will also be processed once a shutdown is initiated but is not guaranteed.
        # Use the shutdown method instead.

    def sample_publisher(self, data):
        vector_object = Vector3Stamped()
        vector_object.header.stamp = rospy.Time.now()
        vector_object.header.frame_id = 'sum'
        vector_object.vector.x = float(data * 1)
        vector_object.vector.y = float(data * 2)
        vector_object.vector.z = float(data * 3)
        self.publisher1_object.publish(vector_object)

    def shutdown(self):
        rospy.loginfo("Beginning clean shutdown routine...")
        time.sleep(5)

        rospy.ServiceProxy(self.lidar_motor_stop_service, Empty)
        rospy.loginfo("Proxied service.")

        # time.sleep(15)

        # perform shutdown tasks here
        rospy.loginfo("Initiating hardware objects in watchdog...")
        myLidar = LIDAR()
        myCar = QCar()

        rospy.loginfo("Terminating hardware objects in watchdog...")
        myCar.terminate()
        myLidar.terminate()
        rospy.loginfo("Terminated hardware objects in watchdog...")

        rospy.loginfo("Shutting down...")


def main(args):
    # args will be a list of commands passed
    optional_nodename = 'sample_node'  # this will be overwritten if a nodename is specified in a launch file
    rospy.init_node(f'{optional_nodename}')
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo(f"{nodename} node started.")  # just log that the node has started.
    hwdn_instance = HardwareWatchDogNode()

    # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
    rospy.on_shutdown(hwdn_instance.shutdown)

    try:
        # run the main functions here
        hwdn_instance.run()
        # rospy.spin()  # only necessary if not publishing (i.e. subscribing only)
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        # this exception block most likely won't be called since there is a shutdown method in the class that will
        # override this and shutdown however is needed but is here just in case.
        rospy.loginfo(f'Encountered {e}. Shutting down.')

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)  # ROS compatible way to handle command line arguments, i.e main(sys.argv)

