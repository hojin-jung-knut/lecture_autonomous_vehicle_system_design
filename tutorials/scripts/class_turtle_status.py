#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, tf
from turtlesim.msg import Pose

class TFBroadCaster():
    def __init__(self):
        rospy.init_node('broadcaster', anonymous=True)
        rospy.Subscriber('turtle1/pose', Pose, self.callback)
        self.status_msg = Pose()
        self.isbroadcast = False
        rospy.spin()

    def callback(self, data):
        self.status_msg = data
        if not self.isbroadcast:
            rospy.loginfo("Broadcasting TF..")
            self.isbroadcast = True
        # 브로드캐스터 생성
        br = tf.TransformBroadcaster()
        # turtle1 상태 tf 브로드캐스팅
        br.sendTransform((self.status_msg.x, self.status_msg.y, 0),
                        tf.transformations.quaternion_from_euler(0,0,self.status_msg.theta),
                        rospy.Time.now(),
                        "turtle",
                        "map")

if __name__ == '__main__':
    try:
        TFBroadCaster()
    except rospy.ROSInternalException:
        pass
