#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import sys

from qcar.q_essential import Camera2D
import numpy as np
import math
import logging
import cv2
import pickle
import time
from std_msgs.msg import String, Float64, Header

from tracker import tracker
import datetime


class Lane_detection_node(object):

    def __init__(self, publish_odometry=True, publish_imu=True):
        super().__init__()

        ## Timing Parameters
        self.sampleRate = 30.0
        self.sampleTime = 1/self.sampleRate

        # -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        # Additional parameters
        self.imageWidth = 320
        self.imageHeight = 240
        # -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        ## Initialize the CSI cameras
        self.myCam1 = Camera2D(camera_id="3", frame_width=self.imageWidth, frame_height=self.imageHeight, frame_rate=self.sampleRate)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('output.avi', fourcc, 30.0, (320, 240))
        # -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

        ## load camera calibration pickle file
        self.calib_result_pickle = pickle.load(open("calibration_pickle_smaller.p", "rb" ))
        self.mtx = self.calib_result_pickle["mtx"]
        self.optimal_camera_matrix = self.calib_result_pickle["optimal_camera_matrix"]
        self.dist = self.calib_result_pickle["dist"]

        ## counter variable 
        self.count = 0
        self.gidx = 0
        self.error_last = 0
        self.yvals_reverse_flag_last =0.

        ## time variables
        self.start = rospy.Time.now()
        self.last_time = self.start

        ## publisher
        error_publisher_queue_size = 1
        self.error_publisher = rospy.Publisher('lane_detection/error/value', Float64,
                                               queue_size=error_publisher_queue_size)
        self.error_publisher_timestamp = rospy.Publisher('lane_detection/error/time', Float64,
                                                         queue_size=error_publisher_queue_size)
        self.value_publisher = rospy.Publisher('lane_detection/error/laneCnenter', Float64, queue_size=error_publisher_queue_size)
                                   

    # -------------------------------------------------------------------------------------------------
    
    def lane_error_calculation(self):
        while not rospy.is_shutdown():
            '''
            current_time = rospy.Time.now()
            dt = (current_time-self.last_time).to_sec()
            rospy.loginfo(f"gap time:{dt}")
            self.last_time = current_time
            '''
            # Capture RGB Image from CSI
            self.myCam1.read()


            # store captured frame in variable
            frame = self.myCam1.image_data.copy()
            # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            self.out.write(frame) 
            # cv2.imshow("Frame", frame)
            # cv2.waitKey(1)

            # undistorted image 
            undistorted_frame = cv2.undistort(frame,self.mtx,self.dist,None,self.mtx)
         
            # filter blues out of image
            mask_img = self.blue_mask(undistorted_frame)
            
            bot_width = 1 # percentage of bottom trapezoidal height
            mid_width =  0.42 # percentage of mid trapezoidal height
            height_pct = 0.68#.58 # percentage of trapezoidal height
            bottom_trim= 0.85 # percentage from top to bottom avoiding the hood of the car

            src = np.float32([[self.imageWidth*(0.5-mid_width/2), self.imageHeight*height_pct],[self.imageWidth*(0.5+mid_width/2),self.imageHeight*height_pct],[self.imageWidth*(0.5+bot_width/2), self.imageHeight*bottom_trim],[self.imageWidth*(0.5-bot_width/2), self.imageHeight*bottom_trim]])
            offset = self.imageHeight*0.25
            dst = np.float32([[offset,0],[self.imageHeight-offset,0],[self.imageHeight-offset,self.imageWidth],[offset,self.imageWidth]])
        
            #perform the warp perspective transform
            M = cv2.getPerspectiveTransform(src,dst)
            Minv = cv2.getPerspectiveTransform(dst,src)
            warped = cv2.warpPerspective(mask_img,M,(self.imageHeight,self.imageWidth),flags=cv2.INTER_LINEAR)

            # show image
            """
            #Visualize the results before/after warping for a birds-eye view along with the source & destination co-ordinate locations
            plt.figure(figsize = (30,20))
            grid = gridspec.GridSpec(8,2)
            # set the spacing between axes.
            grid.update(wspace=0.05, hspace=0.05)  

            plt.subplot(grid[gidx])
            plt.imshow(frame, cmap="gray")
            for i in range(4):
                plt.plot(src[i][0],src[i][1],'rs')
            plt.title('Undistorted Image')

            plt.subplot(grid[gidx+1])
            plt.imshow(warped, cmap="gray")
            for i in range(4):
                plt.plot(dst[i][0],dst[i][1],'rs')
            plt.title('Birds eye view')

            plt.show()
            """

            window_width = 50
            window_height = 80
            
            #set up the overall class to do the lane line tracking
            curve_centers = tracker(Mywindow_width=window_width, Mywindow_height=window_height, Mymargin = 25, My_ym = 10/720, My_xm = 4/384, Mysmooth_factor=15)
            
            window_centroids = curve_centers.find_window_centroids(warped)
            # rospy.loginfo(f"center:{window_centroids}")
            
            # Points used to draw all the left and right windows
            l_points = np.zeros_like(warped)
            r_points = np.zeros_like(warped)
                
            # points used to find the right & left lanes
            rightx = []
            leftx = []

            # Go through each level and draw the windows 
            for level in range(0,len(window_centroids)):
                # Window_mask is a function to draw window areas
                # Add center value found in frame to the list of lane points per left, right
                leftx.append(window_centroids[level][0])
                rightx.append(window_centroids[level][1])

                l_mask = self.window_mask(window_width,window_height,warped,window_centroids[level][0],level)
                r_mask = self.window_mask(window_width,window_height,warped,window_centroids[level][1],level)
                # Add graphic points from window mask here to total pixels found 
                l_points[(l_points == 255) | ((l_mask == 1) ) ] = 255
                r_points[(r_points == 255) | ((r_mask == 1) ) ] = 255

            # Draw the results
            template = np.array(r_points+l_points,np.uint8) # add both left and right window pixels together
            zero_channel = np.zeros_like(template) # create a zero color channel
            template = np.array(cv2.merge((zero_channel,template,zero_channel)),np.uint8) # make window pixels green
            warpage = np.array(cv2.merge((warped,warped,warped)),np.uint8) # making the original road pixels 3 color channels
            result = cv2.addWeighted(warpage, 1, template, 0.5, 0.0) # overlay the original road image with window results
            
            fitting_result = result
            
            
            '''
            # window fitting visualization
            #Visualize the results of the window fitting to lane lines
            plt.imshow(fitting_result, cmap='gray')
            plt.title('Window fitting results')
            plt.show()
            '''


            #fit the lane boundaries to the left, right center positions found
            yvals = range(0,warped.shape[0])  # this is for counter clockwise
            #yvals = yvals[::-1] # this is for clockwise
            
            res_yvals = np.arange(warped.shape[0]-(window_height/2),0,-window_height)
            clockwise_flag = 0
            

            # identify lane is clockwise or counter clockwise
            identity1 = ( (leftx[1]+rightx[1])/2 > (leftx[0]+rightx[0])/2)
            identity2 = ( (leftx[2]+rightx[2])/2 > (leftx[1]+rightx[1])/2)
            identity3 = ( (leftx[3]+rightx[3])/2 > (leftx[2]+rightx[2])/2)
            counts = 0
            if identity1 > 0:
                counts = counts+1
            if identity2 > 0 :
                counts = counts+1
            if identity3 > 0:
                counts = counts+1
            
            if counts >= 2:  # sometimes the lane detction is not very reliable, needs to leave error space
                yvals = yvals[::-1] # this is for clockwise
                clockwise_flag = 1;
            
            left_fitx, right_fitx = self.lane_fit(res_yvals,yvals,leftx,rightx)
                
            
            left_lane = np.array(list(zip(np.concatenate((left_fitx-window_width/2, left_fitx[::-1]+window_width/2),axis=0),np.concatenate((yvals,yvals[::-1]),axis=0))),np.int32)
            right_lane = np.array(list(zip(np.concatenate((right_fitx-window_width/2, right_fitx[::-1]+window_width/2),axis=0),np.concatenate((yvals,yvals[::-1]),axis=0))),np.int32)

            road = np.zeros_like(frame)
            road_bkg = np.zeros_like(frame)
            cv2.fillPoly(road,[left_lane],color=[255,0,0])
            cv2.fillPoly(road,[right_lane],color=[0,0,255])
            cv2.fillPoly(road_bkg,[left_lane],color=[255,255,255])
            cv2.fillPoly(road_bkg,[right_lane],color=[255,255,255])

            road_warped = cv2.warpPerspective(road,Minv,(self.imageWidth,self.imageHeight),flags=cv2.INTER_LINEAR)
            road_warped_bkg= cv2.warpPerspective(road_bkg,Minv,(self.imageWidth,self.imageHeight),flags=cv2.INTER_LINEAR)

            base = cv2.addWeighted(frame,1.0,road_warped, -1.0, 0.0)
            result = cv2.addWeighted(base,1.0,road_warped, 1.0, 0.0)

            ym_per_pix = curve_centers.ym_per_pix # meters per pixel in y dimension
            xm_per_pix = curve_centers.xm_per_pix # meters per pixel in x dimension

            curve_fit_cr = np.polyfit(np.array(res_yvals,np.float32)*ym_per_pix,np.array(leftx,np.float32)*xm_per_pix,2)
            curverad = ((1 + (2*curve_fit_cr[0]*yvals[-1]*ym_per_pix + curve_fit_cr[1])**2)**1.5) /np.absolute(2*curve_fit_cr[0])
            # rospy.loginfo(f"curverad:{warped.shape[0]-(window_height/2)}")
            
            # Calculate the offset of the car on the road
            lane_center = (left_fitx[-1] + right_fitx[-1])/2    # This is lane center
            car_center = warped.shape[1]/2;
            ''''
            rospy.loginfo(f"left:{left_fitx[-1]}")
            rospy.loginfo(f"right:{right_fitx[-1]}")
            rospy.loginfo(f"to left:{car_center-left_fitx[-1]/2}")
            rospy.loginfo(f"to right:{car_center-right_fitx[-1]/2}")
            rospy.loginfo(f"car:{car_center}")
            rospy.loginfo(f"diff:{lane_center-warped.shape[1]/2}")
            '''
            center_diff = (lane_center-car_center)*xm_per_pix  # lateral direction 
            # negative: counter clockwise # positive: clockwise
            
            if clockwise_flag == 0:
                side_pos = 'left'
            else:
                side_pos = 'right'

            #rospy.loginfo(f"center diff:{center_diff}")

            temp = (rospy.Time.now()-self.start).to_sec()


            # draw the text showing curvature, offset & speed
            cv2.putText(result, 'Curve Radius: '+str(round(curverad,3))+'m ',(int((5/600)*self.imageWidth), int((20/338)*self.imageHeight)),cv2.FONT_HERSHEY_SIMPLEX,(float((1.0/600)*self.imageWidth)),(0,0,0),1)
            cv2.putText(result, 'Center Offset: '+str(abs(round(center_diff,3)))+'m '+side_pos+' of center',(int((5/600)*self.imageWidth), int((40/338)*self.imageHeight)), cv2.FONT_HERSHEY_SIMPLEX,(float((1.0/600)*self.imageWidth)),(0,0,0),1)
            
            self.publish_error(center_diff, temp, clockwise_flag, curverad)

            # show lane lines overlayed over original image with geometric text overlayed
            
            # warped --> birds eye view
            # result --> lane overlayed on original image
            # mask_img --> blue image
            # frame --> raw image 
            
            #cv2.imshow('result', result)
            #cv2.waitKey(1)
        


    def lane_fit(self, res_yvals,yvals,leftx, rightx):
        left_fit = np.polyfit(res_yvals, leftx, 1)
        #left_fitx = left_fit[0]*yvals*yvals + left_fit[1]*yvals + left_fit[2]
        left_fitx = left_fit[0]*yvals + left_fit[1]
        left_fitx = np.array(left_fitx,np.int32)
        # rospy.loginfo(f"left fit:{left_fitx[-1]}")
                    
        right_fit = np.polyfit(res_yvals, rightx, 1)
        #right_fitx = right_fit[0]*yvals*yvals + right_fit[1]*yvals + right_fit[2]
        right_fitx = right_fit[0]*yvals + right_fit[1]
        right_fitx = np.array(right_fitx,np.int32)
        # rospy.loginfo(f"right fit:{right_fitx[-1]}")
        return left_fitx, right_fitx
        
    def blue_mask(self,frame):
        # switch to HSV color format to lift blues from image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # blue color mask
        lower_blue = np.array([100,50,50])
        upper_blue = np.array([150,255,255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        return mask
    
    def window_mask(self,width, height, img_ref, center, level):
        output = np.zeros_like(img_ref)
        output[int(img_ref.shape[0]-(level+1)*height):int(img_ref.shape[0]-level*height), max(0,int(center-width)):min(int(center+width),img_ref.shape[1])] = 1
        return output


    '''Publishers'''
    def publish_error(self, error,time, clockwise_flag, curvature):
        # publish the error message
        error_cmd = Float64()
        error_cmd.data = float(error)
        self.error_publisher.publish(error_cmd)

        # publish the time and flag
        error_time_cmd = Float64()
        if clockwise_flag == 0:
            time = -time          # if recive time is a negative value, reverse it and run counter clockwise steering control
        error_time_cmd.data = time
        self.error_publisher_timestamp.publish(error_time_cmd)
        #error_time_cmd.frame_id = 'Image'
        
        # publish lane center value
        lane_center_meg = Float64()
        lane_center_meg.data = curvature
        self.value_publisher.publish(lane_center_meg)
        

def main(args):
    rospy.init_node('lane_detection_node')
    nodename = rospy.get_name()
    rospy.loginfo(f"{nodename} node started")

    r = Lane_detection_node()

    r.lane_error_calculation()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

