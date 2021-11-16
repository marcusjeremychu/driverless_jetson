#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

def handle_camera_pose(msg_stamped):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "left_camera"
    t.transform.translation.x = msg_stamped.pose.pose.position.x
    t.transform.translation.y = msg_stamped.pose.pose.position.y
    t.transform.translation.z = msg_stamped.pose.pose.position.z

    t.transform.rotation.x = msg_stamped.pose.pose.orientation.x
    t.transform.rotation.y = msg_stamped.pose.pose.orientation.y
    t.transform.rotation.z = msg_stamped.pose.pose.orientation.z
    t.transform.rotation.w = msg_stamped.pose.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('camera_broadcaster')
    rospy.Subscriber('/pose',
                     geometry_msgs.msg.PoseWithCovarianceStamped,
                     handle_camera_pose)
    rospy.spin()