#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, tf
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, asin, atan2

class IMUParser:
    def __init__(self):
        rospy.init_node('imu', anonymous=True)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.callback)
        self.br = tf.TransformBroadcaster()

    def callback(self, data):
        quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quat)
        roll_deg = roll/pi*180
        pitch_deg = pitch/pi*180
        yaw_deg = yaw/pi*180
        x,y,z,w = quaternion_from_euler(roll,pitch,yaw)
        roll1 = atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[0]**2+quat[1]**2))/pi*180
        pitch1 = asin(2*(quat[1]*quat[3]-quat[0]*quat[2]))/pi*180
        yaw1 = atan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2))/pi*180

        rospy.loginfo(f'''
        ----------------------[ IMU data(Euler angle) ]----------------------
        roll   (deg) = , {roll_deg}, {roll1}
        pitch  (deg) = , {pitch_deg}, {pitch1}
        yaw    (deg) = , {yaw_deg}, {yaw1})

        ----------------------[ IMU data(Quaternion) ]----------------------
        x = , {data.orientation.x}, {x}
        y = , {data.orientation.y}, {y}
        z = , {data.orientation.z}, {z}
        w = , {data.orientation.w}, {w}

        ----------------------[ IMU data(p,q,r) ]----------------------
        pitch rate = , {data.angular_velocity.y}
        roll rate  = , {data.angular_velocity.x}
        yaw rate  = , {data.angular_velocity.z}

        ----------------------[ IMU data(ax,ay,az) ]----------------------
        ax = , {data.linear_acceleration.x}
        ay = , {data.linear_acceleration.y}
        az = , {data.linear_acceleration.z}
        ''')

if __name__ == '__main__':
    try:
        IMU_parser = IMUParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass