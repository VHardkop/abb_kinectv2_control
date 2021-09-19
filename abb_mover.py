#!/usr/bin/env python3 
import rospy
import geometry_msgs.msg
from move_group_class import MoveGroupPythonInterfaceTutorial
import moveit_commander

def callback(data):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = data.orientation.x
    pose_goal.orientation.y = data.orientation.y
    pose_goal.orientation.z = data.orientation.z
    pose_goal.orientation.w = data.orientation.w
    pose_goal.position.x = data.position.x
    pose_goal.position.y = data.position.y
    pose_goal.position.z = data.position.z
    move_group_object.go_to_pose_goal(pose_goal)
    
def listener():
    rospy.init_node('user_1_hand_follower')
    rospy.Subscriber('kinect2/hand_pose', geometry_msgs.msg.Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    move_group_object = MoveGroupPythonInterfaceTutorial()
    listener()