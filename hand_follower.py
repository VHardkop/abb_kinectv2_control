#!/usr/bin/env python3 
import rospy
import tf
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
from math import ceil
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('user_1_hand_follower')
    rate = rospy.Rate(0.2)
    listener = tf.TransformListener()
    hand_pose = rospy.Publisher('kinect2/hand_pose', geometry_msgs.msg.Pose, queue_size=0)
    hand_pose_stamped = rospy.Publisher('kinect2/hand_pose_stamped', geometry_msgs.msg.PoseStamped, queue_size=0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('base_link', 'kinect/user_1/right_hand', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans_elbow,rot_elbow) = listener.lookupTransform('base_link', 'kinect/user_1/right_elbow', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trans_flange,rot_flange) = listener.lookupTransform('base_link', 'flange', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        hand1 = geometry_msgs.msg.Pose()

        deltax = trans[0] - trans_elbow[0]
        deltay = trans[1] - trans_elbow[1]
        deltaz = trans[2] - trans_elbow[2]

        try:
            yaw = np.arctan(deltay/deltax)
        except:
            yaw = 0
        try:
            roll = -np.arctan(deltaz/deltay)
        except:
            roll = 0
        try:
            pitch = -np.arctan(deltaz/deltax)
        except:
            pitch = 0
        
        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)

        hand1.position.x = round(trans[0] * 1.2, 2)
        hand1.position.y = round(trans[1] * 1.2, 2)
        hand1.position.z = round(trans[2] * 1.2, 2)

        hand1.orientation.x = x
        hand1.orientation.y = y
        hand1.orientation.z = z
        hand1.orientation.w = w

        pose = geometry_msgs.msg.PoseStamped()
        pose.header = std_msgs.msg.Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link" 
        pose.pose = hand1

        hand_pose.publish(hand1)
        hand_pose_stamped.publish(pose)

        rate.sleep()


