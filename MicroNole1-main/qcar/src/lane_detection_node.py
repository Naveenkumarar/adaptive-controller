#!/usr/bin/env python3

"""# Takes in image

# Does stuff to detect lanes

# Outputs center"""


# standard library imports
import sys
import math
import pickle
from pathlib import Path

# third party library imports
import roslib
import rospy, rospkg
import cv2
import numpy as np
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge, CvBridgeError
from tracker import tracker
from pathlib import Path


class LanePublisher:

    def __init__(self, image_publisher_topic_name="lane_detection/rgb/image_color/warped", image_queue_size=1,
                 subscriber_topic_name="/qcar/csi_front", input_encoding="bgr8",
                 output_encoding="mono8", verbose=True, debug=False):

        # initialize Ashwins stuff
        rospack = rospkg.RosPack()
        package_path = Path(rospack.get_path("qcar"))
        ## load camera calibration pickle file
        self.calib_result_pickle = pickle.load(open(package_path / "src/calibration_pickle_WORKING.p", "rb"))
        self.mtx = self.calib_result_pickle["mtx"]
        self.optimal_camera_matrix = self.calib_result_pickle["optimal_camera_matrix"]
        self.dist = self.calib_result_pickle["dist"]

        self.imageWidth = 640
        self.imageHeight = 480

        ## counter variable 
        self.count = 0
        self.gidx = 0
        self.error_last = 0
        self.ts = 0.1
        
        self.ini_time = rospy.Time.now()

        
        # qcar_subscriber_topic_name="/qcar/rgb/image_color"
        # my_subscriber_topic_name="/road_video/image_raw"

        self.image_publisher_topic_name = image_publisher_topic_name
        self.image_queue_size = image_queue_size
        self.image_publish_rate = 30

        self.subscriber_topic_name = subscriber_topic_name
        self.input_encoding = input_encoding
        self.output_encoding = output_encoding

        self.verbose = verbose
        self.debug = debug

        self.lane_image_publisher = rospy.Publisher(self.image_publisher_topic_name, Image,
                                                    queue_size=self.image_queue_size)

        self.lane_fit_publisher = rospy.Publisher("lane_detection/rgb/image_color/fit", Image,
                                                  queue_size=self.image_queue_size)

        error_publisher_queue_size = 1
        self.error_publisher = rospy.Publisher('lane_detection/error/value', Float64,
                                               queue_size=error_publisher_queue_size)
        self.error_publisher_timestamp = rospy.Publisher('lane_detection/error/time', Head,
                                                         queue_size=error_publisher_queue_size)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.subscriber_topic_name, Image, self.callback, queue_size=1)

        if self.verbose:
            print(f"Subscribed to {self.subscriber_topic_name} and publishing to {self.image_publisher_topic_name}")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, self.input_encoding)
        except CvBridgeError as e:
            print(e)

        # undistorted image 
        undistorted_frame = cv2.undistort(cv_image, self.mtx, self.dist, None, self.mtx)

        # filter blues out of image
        mask_img = self.blue_mask(undistorted_frame)

        bot_width = 1  # percentage of bottom trapezoidal height
        mid_width = 0.42  # .32 # percentage of mid trapezoidal height
        height_pct = .58  # percentage of trapezoidal height
        bottom_trim = .85  # percentage from top to bottom avoiding the hood of the car

        src = np.float32([[self.imageWidth * (0.5 - mid_width / 2), self.imageHeight * height_pct],
                          [self.imageWidth * (0.5 + mid_width / 2), self.imageHeight * height_pct],
                          [self.imageWidth * (0.5 + bot_width / 2), self.imageHeight * bottom_trim],
                          [self.imageWidth * (0.5 - bot_width / 2), self.imageHeight * bottom_trim]])
        offset = self.imageHeight * 0.25
        dst = np.float32(
                [[offset, 0], [self.imageHeight - offset, 0], [self.imageHeight - offset, self.imageWidth], [offset, self.imageWidth]])

        # perform the warp perspective transform
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(mask_img, M, (self.imageHeight, self.imageWidth), flags=cv2.INTER_LINEAR)

        window_width = 50
        window_height = 80

        # set up the overall class to do the lane line tracking
        curve_centers = tracker(Mywindow_width=window_width, Mywindow_height=window_height, Mymargin=25, My_ym=10 / 720,
                                My_xm=4 / 384, Mysmooth_factor=15)

        window_centroids = curve_centers.find_window_centroids(warped)

        # Points used to draw all the left and right windows
        l_points = np.zeros_like(warped)
        r_points = np.zeros_like(warped)

        # points used to find the right & left lanes
        rightx = []
        leftx = []

        # Go through each level and draw the windows 
        for level in range(0, len(window_centroids)):
            # Window_mask is a function to draw window areas
            # Add center value found in frame to the list of lane points per left, right
            leftx.append(window_centroids[level][0])
            rightx.append(window_centroids[level][1])

            l_mask = self.window_mask(window_width, window_height, warped, window_centroids[level][0], level)
            r_mask = self.window_mask(window_width, window_height, warped, window_centroids[level][1], level)
            # Add graphic points from window mask here to total pixels found 
            l_points[(l_points == 255) | ((l_mask == 1))] = 255
            r_points[(r_points == 255) | ((r_mask == 1))] = 255

        # Draw the results
        template = np.array(r_points + l_points, np.uint8)  # add both left and right window pixels together
        zero_channel = np.zeros_like(template)  # create a zero color channel
        template = np.array(cv2.merge((zero_channel, template, zero_channel)), np.uint8)  # make window pixels green
        warpage = np.array(cv2.merge((warped, warped, warped)),
                           np.uint8)  # making the original road pixels 3 color channels
        result = cv2.addWeighted(warpage, 1, template, 0.5, 0.0)  # overlay the original road image with window results

        #try:
        #    # to publish a lane image
        #    self.lane_fit_publisher.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        #except (CvBridgeError, rospy.ROSInterruptException) as e:
        #    print(e)

        # fit the lane boundaries to the left, right center positions found
        yvals = range(0, warped.shape[0])

        res_yvals = np.arange(warped.shape[0] - (window_height / 2), 0, -window_height)

        left_fit = np.polyfit(res_yvals, leftx, 1)
        # left_fitx = left_fit[0]*yvals*yvals + left_fit[1]*yvals + left_fit[2]
        left_fitx = left_fit[0] * yvals + left_fit[1]
        left_fitx = np.array(left_fitx, np.int32)

        right_fit = np.polyfit(res_yvals, rightx, 1)
        # right_fitx = right_fit[0]*yvals*yvals + right_fit[1]*yvals + right_fit[2]
        right_fitx = right_fit[0] * yvals + right_fit[1]
        right_fitx = np.array(right_fitx, np.int32)

        left_lane = np.array(list(
            zip(np.concatenate((left_fitx - window_width / 2, left_fitx[::-1] + window_width / 2), axis=0),
                np.concatenate((yvals, yvals[::-1]), axis=0))), np.int32)
        right_lane = np.array(list(
            zip(np.concatenate((right_fitx - window_width / 2, right_fitx[::-1] + window_width / 2), axis=0),
                np.concatenate((yvals, yvals[::-1]), axis=0))), np.int32)

        road = np.zeros_like(cv_image)
        road_bkg = np.zeros_like(cv_image)
        cv2.fillPoly(road, [left_lane], color=[255, 0, 0])
        cv2.fillPoly(road, [right_lane], color=[0, 0, 255])
        cv2.fillPoly(road_bkg, [left_lane], color=[255, 255, 255])
        cv2.fillPoly(road_bkg, [right_lane], color=[255, 255, 255])

        road_warped = cv2.warpPerspective(road, Minv, (self.imageWidth, self.imageHeight), flags=cv2.INTER_LINEAR)
        road_warped_bkg = cv2.warpPerspective(road_bkg, Minv, (self.imageWidth, self.imageHeight), flags=cv2.INTER_LINEAR)

        base = cv2.addWeighted(cv_image, 1.0, road_warped, -1.0, 0.0)
        result = cv2.addWeighted(base, 1.0, road_warped, 1.0, 0.0)

        ym_per_pix = curve_centers.ym_per_pix  # meters per pixel in y dimension
        xm_per_pix = curve_centers.xm_per_pix  # meters per pixel in x dimension

        curve_fit_cr = np.polyfit(np.array(res_yvals, np.float32) * ym_per_pix,
                                  np.array(leftx, np.float32) * xm_per_pix, 2)
        curverad = ((1 + (2 * curve_fit_cr[0] * yvals[-1] * ym_per_pix + curve_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * curve_fit_cr[0])

        # Calculate the offset of the car on the road
        camera_center = (left_fitx[-1] + right_fitx[-1]) / 2
        center_diff = (camera_center - warped.shape[1] / 2) * xm_per_pix
        
        time_send = (rospy.Time.now()-self.ini_time).to_sec()
        center_diff = 0.1

        # publish the error
        #rospy.loginfo(f"Reading error (image): {center_diff}.")
        self.publish_error(center_diff)
        #rospy.loginfo(f"Publishing error (image): {center_diff}. \n")

        side_pos = 'left'
        if center_diff <= 0:
            side_pos = 'right'

        # draw the text showing curvature, offset & speed
        cv2.putText(result, 'Radius of Curvature=' + str(round(curverad, 3)) + 'm ', (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 255, 255), 2)
        cv2.putText(result, 'Vehicle is ' + str(abs(round(center_diff, 3))) + 'm ' + side_pos + ' of center', (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # # show lane lines overlayed over original image with geometric text overlayed
        #
        # # warped --> birds eye view
        # # result --> lane overlayed on original image
        # # mask_img --> blue image
        # # cv_image --> raw image
        #
        # cv2.imshow('result', warped)
        # cv2.waitKey(3)

        # if self.verbose:
        #     print(f'')

        #if self.debug:
        #    cv2.imshow("Image window", warped)
        #    cv2.waitKey(3)

        #try:
        #    # to publish a lane image
        #    self.lane_image_publisher.publish(self.bridge.cv2_to_imgmsg(warped, self.output_encoding))
        #except (CvBridgeError, rospy.ROSInterruptException) as e:
        #    print(e)

    def blue_mask(self, frame):
        # switch to HSV color format to lift blues from image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # blue color mask
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        return mask

    def window_mask(self, width, height, img_ref, center, level):
        output = np.zeros_like(img_ref)
        output[int(img_ref.shape[0] - (level + 1) * height):int(img_ref.shape[0] - level * height),
        max(0, int(center - width)):min(int(center + width), img_ref.shape[1])] = 1
        return output

    def publish_error(self, error):
        # publish the error message
        error_cmd = Float64()
        error_cmd.data = float(error)
        self.error_publisher.publish(error_cmd)
        
        
        #error_cmd2 = Float64()
        #error_cmd2.data = float(error*10)
        #self.error_publisher.publish(error_cmd2)
        
        '''
        # publish the error Header
        error_time_cmd = Float64()
        error_time_cmd.data = time_send
        # error_time_cmd.frame_id = 'Image'
        self.error_publisher_timestamp.publish(error_time_cmd)
        '''

    def shutdown(self):
        pass


def main(args):
    verbose = True
    rospy.init_node('LanePublisher', anonymous=True)
    nodename = rospy.get_name()
    lp = LanePublisher(verbose=verbose)
    # lp.detect_lanes
    try:
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')
        rospy.on_shutdown(lp.shutdown())
        rospy.loginfo("Shutting down")
        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    main(sys.argv)
