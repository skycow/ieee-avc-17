#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
import cv2
import sys
import numpy as np
import time

def region_of_interest(image):
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.1, rows*0.95]
    top_left     = [cols*0.4, rows*0.6]
    bottom_right = [cols*0.9, rows*0.95]
    top_right    = [cols*0.6, rows*0.6]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]],dtype=np.int32)
    mask=np.zeros_like(image)
    if len (image.shape)>2:
	channel_count=image.shape[2]
	ignore_mask_color = (255,)*channel_count
    else:
	ignore_mask_color=255
    cv2.fillPoly(mask,vertices,ignore_mask_color)
   
    return cv2.bitwise_and(image,mask)

def talker():
    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)
    rospy.init_node('safetyoverride', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    lookingforsign = True   # first look for sign, then look for yellow line 
    threshold = 20          # number of consecutive frames that detect a sign
    count = 0		# used to count the frames for the threshold
    pixelsignsize = 180     # will only detect signs larger than this value
    percentofscreen = 5     # threshold to determine if we stop or not
    notStopped = True       # boolean to calculate percentofscreen
    waitTime = 5            # number of seconds to wait at stop sign
    cascPath = "/home/ubuntu/ieee_avc/src/opencvavc/src/stopsign_classifier.xml"
    myCascade = cv2.CascadeClassifier(cascPath)
    video_capture = cv2.VideoCapture(0)

# function used to define a region to look for the yellow line

    while not rospy.is_shutdown():
    # Capture frame-by-frame
      ret, frame = video_capture.read()
      cv2.imshow('Video', frame)
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      if(lookingforsign):	
	    detObjs = myCascade.detectMultiScale(
		gray,
		scaleFactor=1.1,
		minNeighbors=5,
		minSize=(30, 30),
	    )

	    # Draw a rectangle around the detected objects
	    for (x, y, w, h) in detObjs:
		if(w>pixelsignsize and h>pixelsignsize):
		    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
		    count = count+1
	    if(len(detObjs) == 0):
		    count = 0

	    if(count > threshold):
		rospy.loginfo("Found a stop sign.")
                t_end = time.time() + waitTime
                while time.time() < t_end:
                  ack_msg = AckermannDriveStamped()
                  ack_msg.header.stamp = rospy.Time.now()
                  ack_msg.header.frame_id = ''
                  ack_msg.drive.steering_angle = 0
                  ack_msg.drive.speed = 0

                  pub.publish(ack_msg)
                  rate.sleep()
		lookingforsign = False
		count = 0
      else:
	    #convert BGR frame to HSV to easily detect yello
	    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	    
	    #set yellow thresholds
	    lower_yellow=np.array([20,100,100],dtype="uint8")
	    upper_yellow=np.array([30,255,255],dtype="uint8")
	    
	    #create yellow mask
	    yellow=cv2.inRange(hsv,lower_yellow,upper_yellow)
	    mask_yellow=cv2.bitwise_and(gray,yellow)
	    frame = region_of_interest(mask_yellow)
	 
	    #determin if percenofscreen has been reached   
	    if (notStopped):	
	        for pixel in frame:
	            if(cv2.countNonZero(frame)>percentofscreen*.01*frame.size):
		        print("we should stop the car...")
			notStopped = False
			break
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass





# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

#########################################################################
#
# we could also implement torso detection rather easily so we dont 
# start driving after the stop and run into the pedestrian. Just add
# as a command line argument like we did with the stop sign.
#
# On second thought, the laser data should be able to detect and avoid
# the pedestrain.
#
#########################################################################

