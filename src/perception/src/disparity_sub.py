#!/usr/bin/env python
import rospy
import cv2
import os
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage

def cb(msg):
    print("Disparity Received")

rospy.Subscriber("/stereo/disparity", DisparityImage, callback=cb)
rospy.init_node('disparity_sub', anonymous=True) 
rospy.spin()