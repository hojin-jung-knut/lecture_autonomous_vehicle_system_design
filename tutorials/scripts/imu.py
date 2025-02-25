#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import tf
from tf.transformations import euler_from_quaternion
from math import pi

class IMUParser:
    def __init__(self):
        rospy.init_node('imu', anonymous=True)
        self.image_sub = rospy.Subscriber("/imu", Imu, self.callback)
        self.br = tf.TransformBroadcaster()
        rospy.spin()

    def callback(self, data):
        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        roll_deg = roll / pi * 180
        pitch_deg = pitch / pi * 180
        yaw_deg = yaw / pi * 180
        print("-------------")
        print("roll   (deg) = ", roll_deg)
        print("pitch  (deg) = ", pitch_deg)
        print("yaw    (deg) = ", yaw_deg)

        self.prev_time = rospy.get_rostime()

if __name__ == '__main__':
    try:
        IMU_parser = IMUParser()
    except rospy.ROSInterruptException:
        pass
