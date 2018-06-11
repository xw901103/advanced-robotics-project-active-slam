#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry


class Map2OdomTF():
    def __init__(self):
        rospy.init_node('map_tfb', anonymous=True)
        br = tf.TransformBroadcaster()
        self.odom_msg = Odometry()
        rospy.Subscriber('base_pose_ground_truth', Odometry, self.pose_callback)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'odom', 'map')
            pos = self.odom_msg.pose.pose.position
            rot = self.odom_msg.pose.pose.orientation
            br.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w), rospy.Time.now(), 'base_footprint', 'odom')
            rate.sleep()

    def pose_callback(self, msg):
        self.odom_msg = msg


if __name__ == '__main__':
    try:
        tfb = Map2OdomTF()
    except rospy.ROSInterruptException:
        pass