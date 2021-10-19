#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=10)

def imu_callback(msg):
    global pose_pub

    # Build PoseStamped msg
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "camera_imu"
    pose_msg.pose.position.x = 0
    pose_msg.pose.position.y = 0
    pose_msg.pose.position.z = 0
    pose_msg.pose.orientation.x = msg.orientation.x
    pose_msg.pose.orientation.y = msg.orientation.y
    pose_msg.pose.orientation.z = msg.orientation.z
    pose_msg.pose.orientation.w = msg.orientation.w
    
    pose_pub.publish(pose_msg)

if __name__ == "__main__":
    rospy.init_node('pose_pub', anonymous=True) 
    imu_sub = rospy.Subscriber("/imu/data", Imu, callback=imu_callback)
    rospy.spin()


