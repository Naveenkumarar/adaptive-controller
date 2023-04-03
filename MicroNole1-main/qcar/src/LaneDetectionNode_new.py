#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import sys

from qcar.q_essential import Camera2D, Camera3D
import numpy as np
import time
import cv2
from std_msgs.msg import String, Float64, Header

from lane import Lane


class Lane_detection_node(object):

    def __init__(self, publish_odometry=True, publish_imu=True, camera_id = "0"):
        super().__init__()

        ## Timing Parameters
        self.sampleRate = 30.0
        self.sampleTime = 1/self.sampleRate

        # -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        ## Initialize the CSI cameras
        self.myCam1 = Camera2D(camera_id="3", frame_width=320, frame_height=240, frame_rate=self.sampleRate)
        self.myCam2 = Camera3D(frame_width_RGB=320, frame_height_RGB=240, frame_rate_RGB=self.sampleRate)
        
        # opencv
        self.url = "video://localhost:"+ camera_id # address of front csi camera
        self.vid = cv2.VideoCapture(self.url)
        # -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

        self.initime = rospy.Time.now()
        self.current_time = 0.

        # Outputs
        self.lane_curvem, self.center_diff = 0., 0.

        ## publisher
        error_publisher_queue_size = 1
        self.error_publisher = rospy.Publisher('lane_detection/error/value', Float64, queue_size=error_publisher_queue_size)
        self.error_publisher_timestamp = rospy.Publisher('lane_detection/error/time', Float64, queue_size=error_publisher_queue_size)
        self.curvem_publisher = rospy.Publisher('lane_detection/error/laneCnenter', Float64, queue_size=error_publisher_queue_size)
                                   

    # -------------------------------------------------------------------------------------------------
    
    def lane_error_calculation(self):
        while not rospy.is_shutdown():

            self.current_time = (rospy.Time.now() - self.initime).to_sec()

            ## Quanser Capture RGB Image from CSI (information loss problem)
            self.myCam1.read()
            
            # store captured frame in variable
            original_frame = self.myCam1.image_data
            # rospy.loginfo(f"{original_frame.shape}, {type(original_frame)}")
            cv2.imwrite('frame.jpg', original_frame)

            # self.myCam2.read_RGB()
            # original_frame = self.myCam2.image_buffer_RGB           
            
    
            # Create a Lane object
            lane_obj = Lane(orig_frame=original_frame)
    
            # Perform thresholding to isolate lane lines
            # lane_line_markings = lane_obj.get_line_markings()
            lane_line_markings = lane_obj.blue_mask()
    
            # Plot the region of interest on the image
            lane_obj.plot_roi(plot=False)
    
            # Perform the perspective transform to generate a bird's eye view
            # If Plot == True, show image with new region of interest
            warped_frame = lane_obj.perspective_transform(plot=False)
    
            # Generate the image histogram to serve as a starting point
            # for finding lane line pixels
            histogram = lane_obj.calculate_histogram(plot=False)  
        
            if np.any(histogram != 0):
                # Find lane line pixels using the sliding window method 
                left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(plot=False)
    
                # Fill in the lane line
                lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
            
                # Overlay lines on the original frame
                frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
            
                # Calculate lane line curvature (left and right lane lines)
                lane_obj.calculate_curvature(print_to_terminal=False)
            
                # Calculate center offset                                                                 
                lane_obj.calculate_car_position(print_to_terminal=False)
            
                # Display curvature and center offset on image
                frame_with_lane_lines2 = lane_obj.display_curvature_offset(frame=frame_with_lane_lines, plot=False)
            
                # cv2.imshow('detection results',frame_with_lane_lines2)     
                # Display the window until any key is pressed
                # cv2.waitKey(0)
                rospy.loginfo(f"error: {lane_obj.center_offset}")
                self.publish_error(lane_obj.center_offset, lane_obj.center_curvem)
        
        # Close all windows
        # cv2.destroyAllWindows() 
            

    '''Publishers'''
    def publish_error(self,center_offset,center_curvem):
        # publish the center offset message
        error_cmd = Float64()
        error_cmd.data = -1.0 * center_offset
        self.error_publisher.publish(error_cmd)

        # publish the time
        error_time_cmd = Float64()
        error_time_cmd.data = self.current_time
        self.error_publisher_timestamp.publish(error_time_cmd)
        
        # publish lane center curvature value
        lane_center_msg = Float64()
        lane_center_msg.data = center_curvem
        self.curvem_publisher.publish(lane_center_msg)
        

def main(args):
    rospy.init_node('lane_detection_node')
    nodename = rospy.get_name()
    rospy.loginfo(f"{nodename} node started")

    r = Lane_detection_node()

    r.lane_error_calculation()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

