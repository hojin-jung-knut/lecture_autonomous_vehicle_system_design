#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, time, csv, os
import numpy as np
from nav_msgs.msg import Odometry, Path
from math import cos, sin, sqrt
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class DataLogger:
    def __init__(self, pkg_name, path_name):
        rospy.init_node('data_logger', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        self.is_odom = False
        self.forward_point = self.pose = Point()
        self.log_start_time = time.time()
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        self.log_file_path = os.path.join(pkg_path, f"{path_name}.csv")
        self.csv_file = open(self.log_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'pos_x', 'pos_xd', 'pos_y', 'pos_yd'])
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.lfd = 20.0 # Look-ahead distance

    def main(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_odom:
                self.forward_path()
            else:
                rospy.loginfo("Unable to subscribe to '/odom' topic")
            rate.sleep()

    def forward_path(self):
        translation = [self.pose.x, self.pose.y]
        transformation_matrix = self.get_transformation_matrix(translation)
        inv_transformation_matrix = np.linalg.inv(transformation_matrix)

        self.is_look_forward_point = False
        for pose in self.path.poses:
            local_path_point = self.transform_point(inv_transformation_matrix, pose.pose.position)

            if local_path_point[0] > 0:
                distance = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                if distance >= self.lfd:
                    self.forward_point = pose.pose.position
                    self.is_look_forward_point = True
                    break

    def transform_point(self, transformation_matrix, point):
        global_path_point = np.array([point.x, point.y, 1])
        local_path_point = transformation_matrix.dot(global_path_point)
        return local_path_point
    
    def get_transformation_matrix(self, translation):
        return np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        self.pose = msg.pose.pose.position
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        if (self.last_pose_x - self.pose.x)**2 + (self.last_pose_y - self.pose.y)**2 > 1:
            print(f"x: {self.pose.x:.2f}, y: {self.pose.y:.2f}")
            log = np.around([time.time() - self.log_start_time, self.pose.x,
                             self.forward_point.x, self.pose.y, self.forward_point.y], 4)
            self.csv_writer.writerow(log)
            self.last_log_time = time.time()
            self.last_pose_x = self.pose.x
            self.last_pose_y = self.pose.y

    def stop(self):
        print("Logging finished")
        self.csv_file.close()

if __name__ == '__main__':
    try:
        data_logger = DataLogger("tutorials", "term_project_studentnumber")
        data_logger.main()
    except rospy.ROSInterruptException:
        pass
    finally:
        data_logger.stop()
