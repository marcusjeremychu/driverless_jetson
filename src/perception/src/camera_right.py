#!/usr/bin/env python
import rospy
import cv2
import os
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import camera_left

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
FRAMERATE = 59.999999

def gstreamer_pipeline_0(
    capture_width=IMAGE_WIDTH,
    capture_height=IMAGE_HEIGHT,
    display_width=IMAGE_WIDTH,
    display_height=IMAGE_HEIGHT,
    framerate=FRAMERATE,                                                            
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor_id=0 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink "
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
    cap_0 = cv2.VideoCapture(gstreamer_pipeline_0(flip_method=2), cv2.CAP_GSTREAMER)
    vid_pub_0 = rospy.Publisher("/stereo/right/image_raw", Image, queue_size=1)
    info_pub = rospy.Publisher("/stereo/right/camera_info", CameraInfo, queue_size=1)
    info_msg = camera_left.load_camera_yaml('/home/uwfsae/driverless_ws/src/perception/camera_calibration_parameters/right.yaml')

    rate = rospy.Rate(30)
    bridge = CvBridge()
    
    if cap_0.isOpened():
        while not rospy.core.is_shutdown():
            try:
                ret0, frame0 = cap_0.read()

                if ret0:
                    img0 = bridge.cv2_to_imgmsg(frame0, 'passthrough')

                    # Stamps must match to be synchronized
                    stamp = rospy.Time.now()
                    img0.header.stamp = stamp
                    img0.header.frame_id="right_camera"
                    info_msg.header.stamp = stamp
                    info_msg.header.frame_id="right_camera"

                    img0.encoding="bgr8"
                    img0.width = IMAGE_WIDTH
                    img0.height = IMAGE_HEIGHT
                    vid_pub_0.publish(img0)
                    info_pub.publish(info_msg)
            except KeyboardInterrupt:
                break
            rate.sleep()
        cap_0.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == '__main__':
    rospy.init_node('camera_right', anonymous=True) 
    stream_video()