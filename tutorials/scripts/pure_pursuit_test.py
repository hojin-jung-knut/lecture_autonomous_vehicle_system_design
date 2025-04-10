#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np
from math import cos, sin, sqrt, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
from tf.transformations import euler_from_quaternion

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.local_path_sub = rospy.Subscriber("/local_path", Path, self.path_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2 # Velocity control type

        self.is_path = self.is_odom = False
        self.forward_point = self.current_position = Point()

        self.vehicle_length = 3.0
        self.lfd = 5.0

        self.log_start_time = time.time()
        self.last_log_time = 0.0

    def main(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                self.pure_pursuit_control()
            else:
                if not self.is_path:
                    rospy.loginfo("Unable to subscribe to '/local_path' topic")
                if not self.is_odom:
                    rospy.loginfo("Unable to subscribe to '/odom' topic")

            rate.sleep()

    def pure_pursuit_control(self):
        vehicle_position = self.current_position
        cos_yaw = cos(self.vehicle_yaw)
        sin_yaw = sin(self.vehicle_yaw)
        translation_matrix = np.array([
            [cos_yaw, -sin_yaw, vehicle_position.x],
            [sin_yaw, cos_yaw, vehicle_position.y],
            [0, 0, 1]
        ])
        inv_translation_matrix = np.linalg.inv(translation_matrix)

        self.is_look_forward_point = False
        for path_point in self.path.poses:
            global_path_point = np.array([path_point.pose.position.x, path_point.pose.position.y, 1])
            local_path_point = inv_translation_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                distance = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                if distance >= self.lfd:
                    self.forward_point = path_point.pose.position
                    self.is_look_forward_point = True
                    break

        if self.is_look_forward_point:
            theta = atan2(local_path_point[1], local_path_point[0])
            self.ctrl_cmd_msg.steering = atan2((2 * sin(theta) * self.vehicle_length), self.lfd)
            self.ctrl_cmd_msg.velocity = 20.0

            rospy.loginfo(f'''
                -------------------------------------
                "steering (deg) = {self.ctrl_cmd_msg.steering * 180 / pi:.2f}
                "velocity (kph) = {self.ctrl_cmd_msg.velocity:.2f}
                -------------------------------------''')   
        else:
            self.ctrl_cmd_msg.steering = self.ctrl_cmd_msg.velocity = 0.0
            print("Forward point not found")

        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position = msg.pose.pose.position

if __name__ == '__main__':
    try:
        pure_pursuit = PurePursuit()
        pure_pursuit.main()
    except rospy.ROSInterruptException:
        pass
