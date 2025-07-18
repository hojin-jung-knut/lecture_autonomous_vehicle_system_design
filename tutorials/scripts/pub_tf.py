#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, tf
from nav_msgs.msg import Odometry

class PubTF:
    def __init__(self):
        rospy.init_node('pub_tf', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.callback)
        self.is_odom = False
        rospy.spin()

    def callback(self, msg):
        self.is_odom = True
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        br = tf.TransformBroadcaster()
        br.sendTransform(
            (position.x, position.y, 1),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            rospy.Time.now(),
            "Ego",
            "map"
        )
        rospy.loginfo(f"Broadcasting Transform: Position({position.x}, {position.y}) Orientation({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})")

if __name__ == '__main__':
    try:
        PubTF()
    except rospy.ROSInterruptException:
        pass
