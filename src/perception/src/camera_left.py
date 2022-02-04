#!/usr/bin/env python
import rospy
import cv2
import os
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import yaml

IMAGE_WIDTH = 1280  
IMAGE_HEIGHT = 720
FRAMERATE = 59.999999

def load_camera_yaml(filename):
    fp = open(filename, "r")
    camera_info_yaml = yaml.safe_load(fp)

    cameraInfo = CameraInfo()
    cameraInfo.height = camera_info_yaml['image_height']
    cameraInfo.width = camera_info_yaml['image_width']
    cameraInfo.distortion_model = camera_info_yaml['camera_model']
    cameraInfo.K = camera_info_yaml['camera_matrix']['data']
    cameraInfo.D = camera_info_yaml['distortion_coefficients']['data']
    cameraInfo.R = camera_info_yaml['rectification_matrix']['data']
    cameraInfo.P = camera_info_yaml['projection_matrix']['data']
    return cameraInfo


def gstreamer_pipeline_1(
    capture_width=IMAGE_WIDTH,
    capture_height=IMAGE_HEIGHT,
    display_width=IMAGE_WIDTH,
    display_height=IMAGE_HEIGHT,
    framerate=FRAMERATE,                                                            
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor_id=1 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def stream_video():
    # 0 is right, 1 is left
    cap_1 = cv2.VideoCapture(gstreamer_pipeline_1(flip_method=2), cv2.CAP_GSTREAMER)
    vid_pub_1 = rospy.Publisher("/stereo/left/image_raw", Image, queue_size=1)
    info_pub = rospy.Publisher("/stereo/left/camera_info", CameraInfo, queue_size=1)
    info_msg = load_camera_yaml('/home/uwfsae/driverless_ws/src/perception/camera_calibration_parameters/left.yaml')

    rate = rospy.Rate(20)
    bridge = CvBridge()
    
    if cap_1.isOpened():
        
        while not rospy.core.is_shutdown():
            try:
                ret1, frame1 = cap_1.read()

                if ret1:
                    img1 = bridge.cv2_to_imgmsg(frame1, 'passthrough')

                    # Stamps must match to be synchronized
                    stamp = rospy.Time.now()
                    img1.header.stamp = stamp
                    img1.header.frame_id="left_camera"
                    info_msg.header.stamp = stamp
                    info_msg.header.frame_id="left_camera"

                    img1.encoding="bgr8"
                    img1.width = IMAGE_WIDTH
                    img1.height = IMAGE_HEIGHT
                    vid_pub_1.publish(img1)
                    info_pub.publish(info_msg)  

            except KeyboardInterrupt:
                break
            rate.sleep()
        cap_1.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == '__main__':
    rospy.init_node('camera_left', anonymous=True) 
    stream_video()