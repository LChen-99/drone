#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import math
import rospy
import threading
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf_conversions

class TagPub(object):

    def __init__(self):
        # Set point publishing MUST be faster than 2Hz

       
        self.pub_tagpose_topic = "/tag_world"
        self.rate = rospy.Rate(10)
        self.tag_pose = PoseStamped()
        self.pub_tagpose = rospy.Publisher(self.pub_tagpose_topic, PoseStamped, queue_size=10, tcp_nodelay=True)
        time.sleep(1)

    @staticmethod
    def ros_spin():
        rospy.spin()


    def spin(self):
        while not rospy.is_shutdown():
            
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            self.tag_pose.pose.orientation.x = q[0]
            self.tag_pose.pose.orientation.y = q[1]
            self.tag_pose.pose.orientation.z = q[2]
            self.tag_pose.pose.orientation.w = q[3]
            self.tag_pose.pose.position.x = 0.5
            self.tag_pose.pose.position.y = 0
            self.tag_pose.pose.position.z = 0
            self.pub_tagpose.publish(self.tag_pose)
        
            self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node("tagpose_node_py")
    test = TagPub()
    test.spin()
