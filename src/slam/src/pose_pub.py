#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import numpy as np

pose_pub = rospy.Publisher("pose", PoseWithCovarianceStamped, queue_size=10)

def imu_callback(msg):
    global pose_pub
    covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -1, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -1, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -1, 0.0, 0.0, 0.0, 0.0, 0.0,]

    # Build PoseStamped msg
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "left_camera"
    pose_msg.pose.pose.position.x = 0
    pose_msg.pose.pose.position.y = 0
    pose_msg.pose.pose.position.z = 0
    pose_msg.pose.pose.orientation.x = msg.orientation.x
    pose_msg.pose.pose.orientation.y = msg.orientation.y + 0.207 # corrects a bias, probably due to magnetometer's hard iron correction
    pose_msg.pose.pose.orientation.z = msg.orientation.z
    pose_msg.pose.pose.orientation.w = msg.orientation.w
    # pose_msg.pose.covariance = covariance
    
    pose_pub.publish(pose_msg)

if __name__ == "__main__":
    rospy.init_node('pose_pub', anonymous=True) 
    imu_sub = rospy.Subscriber("/imu/data", Imu, callback=imu_callback)
    rospy.spin()


