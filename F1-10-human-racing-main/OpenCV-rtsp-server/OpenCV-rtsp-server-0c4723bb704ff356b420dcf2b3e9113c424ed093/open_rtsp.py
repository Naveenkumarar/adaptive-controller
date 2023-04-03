#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  5 03:11:31 2020

@author: prabhakar
"""

import cv2

#scale_percent = 200 # percent of original size
cv2.namedWindow("RTSP View", cv2.WINDOW_NORMAL)
cap = cv2.VideoCapture("rtsp://192.168.2.13:8554/video_stream")
while True:
    
    ret, frame = cap.read()
    if ret:
        #width = int(frame.shape[1] * scale_percent / 100)
        #height = int(frame.shape[0] * scale_percent / 100)
        #dim = (width, height)
        #resized_frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        
        cv2.imshow("RTSP View", frame)
        cv2.waitKey(1)
    else:
        print("unable to open camera")
        break
cap.release()
cv2.destroyAllWindows()


