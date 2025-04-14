#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, time, csv, os
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point
from math import sqrt

class Road:
    def __init__(self, pkg_name, path_name):
        rospy.init_node('curvature', anonymous=True)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.log_start_time = time.time()
        self.last_log_time = 0.0
        self.curvature = 0.0
        self.pose = Point()
        self.is_global_path = self.is_odom = False
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        self.log_file_path = os.path.join(pkg_path, f"{path_name}.csv")
        self.csv_file = open(self.log_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'curvature'])
        self.curv = Curvature()

        while not rospy.is_shutdown():
            rate = rospy.Rate(30)  # 30Hz
            if self.is_global_path:
                self.curvature = self.curv.path_curvature(self.global_path, self.pose, 50)
                rospy.loginfo('Path data received')
            else:
                rospy.loginfo('Waiting for global path data')
            if time.time() - self.last_log_time > 1:
                log = np.around([time.time()-self.log_start_time, self.curvature], 4)
                self.csv_writer.writerow(log)
                self.last_log_time = time.time()    
            self.reset_flags()
            rate.sleep()

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        self.is_odom = True

    def reset_flags(self):
        self.is_global_path = self.is_odom = False

    def stop(self):
        print("Logging finished")
        self.csv_file.close()

class Curvature:
    def __init__(self):
        self.i = 0

    def path_curvature(self, global_path, pose, point_num):
        print(len(global_path.poses))
        for i in range(1, len(global_path.poses)-1):
            if ((pose.x-global_path.poses[i].pose.position.x)**2 + (pose.y-global_path.poses[i].pose.position.y)**2) < ((pose.x-global_path.poses[self.i].pose.position.x)**2 + (pose.y-global_path.poses[self.i].pose.position.y)**2):
                self.i = i
        print(self.i)
        x_list = []
        y_list = []
        for box in range(-point_num, point_num):
            x = global_path.poses[self.i + box].pose.position.x
            y = global_path.poses[self.i + box].pose.position.y
            x_list.append([-2*x, -2*y, 1])
            y_list.append(-x**2 - y**2)

        x_matrix = np.array(x_list)
        y_matrix = np.array(y_list)
        x_trans = x_matrix.T

        a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
        a, b, c = a_matrix

        return sqrt(a**2 + b**2 - c)

if __name__ == '__main__':
    try:
        road = Road("tutorials", "log_curvature")
    except rospy.ROSInterruptException:
        pass
    finally:
        road.stop()
