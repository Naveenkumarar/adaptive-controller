#!/usr/bin/env python3

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import sys

# third-party library imports
import roslib
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# import local modules
from qcar.q_essential import Camera2D


class CSINode(object):
    def __init__(self, camera_direction='f', image_height=240, image_width=320, sample_rate=30.0, publish_image=True, display_image=False):
        """
        :param camera_direction: f-front, r-right, l-left, b-back. Examples, camera_direction='fbl' for front, back,
                left cameras
        """
        super().__init__()
        #rospy.on_shutdown(self.shutdown)

        self.imageWidth = image_width
        self.imageHeight = image_height
        self.sampleRate = sample_rate

        self.camera_direction = camera_direction
        self.number_of_cameras = 0
        self.publish_image = publish_image
        self.display_image = display_image

        if 'f' in self.camera_direction:
            # front CSI camera
            self.cam_pub_f = rospy.Publisher('/qcar/csi_front', Image, queue_size=10)
            self.csi4 = Camera2D(camera_id="3", frame_width=self.imageWidth, frame_height=self.imageHeight,
                                 frame_rate=self.sampleRate)
            self.number_of_cameras += 1

        if 'b' in self.camera_direction:
            self.cam_pub_b = rospy.Publisher('/qcar/csi_back', Image, queue_size=10)
            self.csi2 = Camera2D(camera_id="1", frame_width=self.imageWidth, frame_height=self.imageHeight,
                                 frame_rate=self.sampleRate)
            self.number_of_cameras += 1

        if 'r' in self.camera_direction:
            self.cam_pub_r = rospy.Publisher('/qcar/csi_right', Image, queue_size=10)
            self.csi1 = Camera2D(camera_id="0", frame_width=self.imageWidth, frame_height=self.imageHeight,
                                 frame_rate=self.sampleRate)
            self.number_of_cameras += 1

        if 'l' in self.camera_direction:
            self.cam_pub_l = rospy.Publisher('/qcar/csi_left', Image, queue_size=10)
            self.csi3 = Camera2D(camera_id="2", frame_width=self.imageWidth, frame_height=self.imageHeight,
                                 frame_rate=self.sampleRate)
            self.number_of_cameras += 1

        self.bridge = CvBridge()

    def read_image(self):
        if 'r' in self.camera_direction:
            self.csi1.read()
        if 'b' in self.camera_direction:
            self.csi2.read()
        if 'l' in self.camera_direction:
            self.csi3.read()
        if 'f' in self.camera_direction:
            self.csi4.read()

    def run(self):
        while not rospy.is_shutdown():
            self.read_image()

            if self.publish_image:
                # self.rate_pub.publish(msg)
                if 'r' in self.camera_direction:
                    self.process_cam_data(self.cam_pub_r, self.csi1.image_data)
                if 'b' in self.camera_direction:
                    self.process_cam_data(self.cam_pub_b, self.csi2.image_data)
                if 'l' in self.camera_direction:
                    self.process_cam_data(self.cam_pub_l, self.csi3.image_data)
                if 'f' in self.camera_direction:
                    self.process_cam_data(self.cam_pub_f, self.csi4.image_data)

    # --------------------------------------------------------------------------------------------------------------
    def process_cam_data(self, cam_info, img_data):
        #rospy.loginfo(f"Image data: {type(img_data)}, Image shape: {img_data.shape}")
        pub_img = self.bridge.cv2_to_imgmsg(img_data, "bgr8")
        pub_img.header.stamp = rospy.Time.now()
        pub_img.header.frame_id = 'cam_img_input'
        if self.display_image:
            cv2.imshow(str(cam_info), img_data)
            cv2.waitKey(1)

    def shutdown(self):
        rospy.loginfo("Beginning clean shutdown routine...")
        # perform shutdown tasks here
        if 'r' in self.camera_direction:
            self.csi1.terminate()
        if 'b' in self.camera_direction:
            self.csi2.terminate()
        if 'l' in self.camera_direction:
            self.csi3.terminate()
        if 'f' in self.camera_direction:
            self.csi4.terminate()
        rospy.loginfo("Shutting down CSI cameras...")


def main(args):
    rospy.init_node('csi_node')  # this will be overwritten if a nodename is specified in a launch file
    nodename = rospy.get_name()
    rospy.loginfo(f"{nodename} node started.")
    csi_instance = CSINode()
    rospy.on_shutdown(csi_instance.shutdown)

    try:
        # run the main functions here
        csi_instance.run()
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
    main(sys.argv)
