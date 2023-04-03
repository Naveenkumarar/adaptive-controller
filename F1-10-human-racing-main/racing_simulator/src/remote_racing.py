#!/usr/bin/env python3

"""
Publishes the following:
1. publish requried joystick command to MicroNole

Subscribes to the following:
1. subscribe racing cockpit command
2. subscribe camera infor from MicroNole (192.168.2.15)

Functions:
1. display the camera view
"""

import time

import numpy as np
import math
import rospy
# from multimaster_udp.transport import UDPSubscriber

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy, Image, CompressedImage
#import ros_numpy
#from cv_bridge import CvBridge, CvBridgeError
import cv2
import pickle
   


class racingNode(object):
    """docstring for ClassName"""

    def __init__(self, publish_joy=True, publish_rate= 120.):
        """Constructor for racing simulator"""
        super().__init__()

        self.publish_rate = publish_rate  # /G29/joy topic miximal rate: 35hz
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        self.ctrl_pub = rospy.Publisher('/racing_cockpit/ctrl_cmd', Vector3Stamped, queue_size=10)
        self.ctrl_sub = rospy.Subscriber("/G29/joy", Joy, self.ctrl_callback, queue_size=10)
        #self.video_sub = rospy.Subscriber("/qcar/camera", Image, self.MicroNole_camera_callback, queue_size=1, buff_size=1920*1080*3)
        # self.video_sub_compressed = rospy.Subscriber("/qcar/compressed_camera", CompressedImage, self.compreessedIMG_callback, queue_size=1, buff_size=2*1920*1080*3)
        # self.video_sub_compressed = UDPSubscriber("/qcar/compressed_camera", CompressedImage, self.compreessedIMG_callback, queue_size=1)
        # self.timer_publisher = rospy.Timer(rospy.Duration(1/120), self.timer_watcher, reset=False)
        self.delay = 0.0
        
        self.axes = np.zeros(6)
        self.button = np.zeros(18)
        self.command = np.zeros(3)
        self.throttle_scale, self.steering_scale = 0.2/2, math.pi/2

        #self.bridge = CvBridge()

        # image parameters  (keep same as ones set on MicroNole)
        #self.imageWidth = 320
        #self.imageHeight = 240
        #self.cv_image = np.zeros((self.imageHeight,self.imageWidth,3), np.uint8)


    def publish_data(self):
        while not rospy.is_shutdown():
            # all racing cockpit joystick command
            self.steering = self.axes[0]     # +:left, -:right [-1,1]
            self.throttle = self.axes[1]+1   # [-1,1] --> [0,2] default:-1
            self.brake    = self.axes[2]+1   # [-1,1] --> [0,2] default:-1
            self.clutch   = self.axes[3]     # [-1,1]
            if self.axes[4] > 0.0:
                self.panel_left = 1
                self.panel_right = 0
            else:
                self.panel_left = 0
                self.panel_right = 1
                
            if self.axes[5] > 0.0:
                self.panel_up = 1
                self.panel_down = 0
            else:
                self.panel_up = 0
                self.panel_down = 1
            
            self.A, self.B, self.X, self.Y = self.button[:4]
            self.shift_paddle_right = self.button[4]
            self.shift_paddle_left = self.button[5]
            self.RSB = self.button[8]
            self.LSB = self.button[9]
            self.xbox = self.button[10]
            self.gearshift_left_f = self.button[12]
            self.gearshift_middle_f = self.button[14]
            self.gearshift_right_f = self.button[16]
            self.gearshift_left_b = self.button[13]
            self.gearshift_middle_b = self.button[15]
            self.gearshift_right_b = self.button[17]
            
            # publish requried command to MicroNodel
            self.command[0] = np.maximum(np.minimum(self.steering_scale * self.steering,math.pi/6),-math.pi/6)
            self.command[1] = np.maximum(self.throttle_scale * ( np.abs(self.throttle) - np.abs(self.brake) ), 0)
            self.command[2] = 0
            if self.gearshift_left_f==1 or self.gearshift_middle_f==1 or self.gearshift_right_f==1:
                self.command[2] = 1
            if self.gearshift_left_b==1 or self.gearshift_middle_b==1 or self.gearshift_right_b==1:
                self.command[2] = -1
            self.publisher_joy()
            
            '''
            # subscribe the camera view
            (rows,cols,channels) = self.cv_image.shape
            # undistorted_frame = cv2.undistort(self.cv_image,self.mtx,self.dist,None,self.mtx)  # undistorted image 
            cv2.imshow("Racing view", self.cv_image)
            cv2.waitKey(1)
            '''

            # self.rate.sleep()


    def ctrl_callback(self, data):
        self.axes = data.axes
        self.button = data.buttons
    '''
    def MicroNole_camera_callback(self,data):
        # self.cv_image = ros_numpy.numpify(data)
        now = rospy.Time.now()
        timestamp = data.header.stamp
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        delay = (now - timestamp).to_sec()
        rospy.loginfo(f"Delay: {delay} \n")
    
        
    def compreessedIMG_callback(self,ros_data):
        self.delay = (rospy.Time.now() - ros_data.header.stamp).to_sec()
        #rospy.loginfo(f"Delay: {delay}")
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # rospy.loginfo(f"subscribe the image from MicroNole {self.cv_image}")
    '''
        
    
    def publisher_joy(self):
        joy_command = Vector3Stamped()
        joy_command.header.stamp = rospy.Time.now()
        joy_command.header.frame_id = 'remote racing'
        joy_command.vector.x = float(self.command[0])
        joy_command.vector.y = float(self.command[1])
        joy_command.vector.z = float(self.command[2])
        self.ctrl_pub.publish(joy_command)

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
        
        rospy.loginfo("Shutting down cleanly...")
        
    def timer_watcher(self, event):
        rospy.loginfo(f"Delay: {self.delay}")


if __name__ == '__main__':
    rospy.init_node('racingNode')
    racing = racingNode()
    # use rospy.on_shutdown() to perform interpolation and error detection
    rospy.on_shutdown(racing.shutdown)

    try:
        racing.publish_data()
        # rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')
        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


