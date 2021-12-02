#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import numpy as np

pose_pub = rospy.Publisher("pose", PoseWithCovarianceStamped, queue_size=10)

def quaternion_multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def imu_callback(msg):
    global pose_pub
    covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -1, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -1, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -1, 0.0, 0.0, 0.0, 0.0, 0.0,]

    q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    # Build PoseStamped msg
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "left_camera"
    pose_msg.pose.pose.position.x = 0
    pose_msg.pose.pose.position.y = 0
    pose_msg.pose.pose.position.z = 0
    pose_msg.pose.pose.orientation.x = q[0]
    pose_msg.pose.pose.orientation.y = q[1]
    pose_msg.pose.pose.orientation.z = q[2]
    pose_msg.pose.pose.orientation.w = q[3]
    # pose_msg.pose.covariance = covariance
    
    pose_pub.publish(pose_msg)

if __name__ == "__main__":
    rospy.init_node('pose_pub', anonymous=True) 
    imu_sub = rospy.Subscriber("/imu/data", Imu, callback=imu_callback)
    rospy.spin()


