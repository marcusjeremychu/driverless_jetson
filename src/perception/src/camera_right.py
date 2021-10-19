#!/usr/bin/env python
import rospy
import cv2
import os
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

def gstreamer_pipeline_0(
    capture_width=1640,
    capture_height=1232,
    display_width=1640,
    display_height=1232,
    framerate=29.999999,                                                            
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
    vid_pub_0 = rospy.Publisher("/vid_0", Image, queue_size=1)

    rate = rospy.Rate(30)
    bridge = CvBridge()
    
    if cap_0.isOpened():
        while not rospy.core.is_shutdown():
            try:
                ret0, frame0 = cap_0.read()

                if ret0:
                    img0 = bridge.cv2_to_imgmsg(frame0, 'passthrough')
                    vid_pub_0.publish(img0)
            except KeyboardInterrupt:
                break
            rate.sleep()
        cap_0.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == '__main__':
    rospy.init_node('camera', anonymous=True) 
    stream_video()