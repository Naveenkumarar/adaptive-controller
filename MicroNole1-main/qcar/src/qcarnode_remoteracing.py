#!/usr/bin/env python3

"""
Publishes the following:
1. publish camera image to (192.168.2.19)

Subscribes to the following:
1. subscribe joystick command from (192.168.2.19)
2. 

Functions:
1. write command to MicroNole
"""

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import time
import sys

# third-party library imports
import numpy as np
import roslib
import rospy
import ros_numpy
# from multimaster_udp.transport import UDPPublisher

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image, CompressedImage

#from cv_bridge import CvBridge, CvBridgeError
import cv2

# import local modules
from qcar.product_QCar import QCar
from qcar.q_essential import Camera2D


class RacingNode(object):
    def __init__(self, publish_rate = 480.0):
        super().__init__()

        self.command_sub = rospy.Subscriber('/racing_cockpit/ctrl_cmd', Vector3Stamped, self.racing_command, queue_size=10)
        #self.image_pub = rospy.Publisher("/qcar/camera", Image, queue_size=1)
        #self.image_pub_compressed = rospy.Publisher("/qcar/compressed_camera", CompressedImage, queue_size=1)
        # self.image_pub_compressed = UDPPublisher("/qcar/compressed_camera", CompressedImage, queue_size=1)

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = publish_rate  #
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

        # initialization of variables
        self.steering, self.throttle = 0.0, 0.0
        self.myCar = QCar()
        
        '''
        # image pub
        self.bridge = CvBridge()

        self.imageWidth = 320
        self.imageHeight = 240

        self.sampleRate = 120.0
        self.sampleTime = 1/self.sampleRate
        self.myCam1 = Camera2D(camera_id="3", frame_width=self.imageWidth, frame_height=self.imageHeight, frame_rate=self.sampleRate)
        '''

    def loop(self):
        while not rospy.is_shutdown():
            '''
            ## publish camera view
            self.myCam1.read()   # Capture RGB Image from CSI
            frame = self.myCam1.image_data   # store captured frame in variable
            
            # 1. cv_bridge option
            #image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            #image_msg.header.stamp = rospy.Time.now()
            # self.image_pub.publish(image_msg)
            #self.image_pub.publish(ros_numpy.msgify(Image,frame,encoding='bgr8'))
            
            # 2. compressed image option
            compressed_img = CompressedImage()
            compressed_img.header.stamp = rospy.Time.now()
            compressed_img.format = "jpeg"
            compressed_img.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            self.image_pub_compressed.publish(compressed_img)
            '''
            
            
            ## write commands
            self.command = np.array([self.throttle, self.steering])
            LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            if self.steering > 0.3:
                LEDs[0] = 1
                LEDs[2] = 1
            elif self.steering < -0.3:
                LEDs[1] = 1
                LEDs[3] = 1
            if self.steering < 0:
                LEDs[5] = 1

            current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command, LEDs)

            # self.rate.sleep()

    # --------------------------------------------------------------------------------------------
    def racing_command(self,data):
        self.steering = data.vector.x
        self.throttle = 0.0
        if data.vector.z > 0.5:
            self.throttle = data.vector.y
        if data.vector.z < -0.5:
            self.throttle = -1.0 * data.vector.y

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
        # perform shutdown tasks here
        command = np.array([0.0, 0.0])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        current, batteryVoltage, encoderCounts = self.myCar.read_write_std(command, LEDs)
        rospy.loginfo("Shutting down cleanly...")


if __name__ == '__main__':
    rospy.init_node('racingNode')
    racing = RacingNode()
    # use rospy.on_shutdown() to perform interpolation and error detection
    rospy.on_shutdown(racing.shutdown)

    try:
        racing.loop()
        # rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')
        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)
