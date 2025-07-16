#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, time, csv, os
import numpy as np
from nav_msgs.msg import Odometry, Path
from math import cos, sin, sqrt
from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus
from tf.transformations import euler_from_quaternion

class DataLogger:
    def __init__(self, pkg_name, path_name):
        rospy.init_node('data_logger', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.is_look_forward_point = self.is_path = self.is_odom = self.is_status = False
        self.forward_point, self.pose = Point(), Point()
        self.log_start_time = time.time()
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        self.log_file_path = os.path.join(pkg_path, f"{path_name}.csv")
        self.csv_file = open(self.log_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'xd', 'y', 'yd', 'vx(km/h)', 'wheel_angle(deg)', 'accel', 'brake'])
        self.last_pose_x, self.last_pose_y = 0.0, 0.0
        self.lfd = 20.0 # Look-ahead distance

    def main(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_odom == True and self.is_path == True and self.is_status == True:
                self.forward_path()
                if self.is_look_forward_point == True and (self.last_pose_x - self.pose.x)**2 + (self.last_pose_y - self.pose.y)**2 > 1:
                    log = np.around([time.time() - self.log_start_time, self.pose.x, self.forward_point.x, self.pose.y, 
                             self.forward_point.y, self.ego.velocity.x*3.6, self.ego.wheel_angle, self.ego.accel, self.ego.brake], 4)
                    self.csv_writer.writerow(log)
                    self.last_pose_x, self.last_pose_y = self.pose.x, self.pose.y 
            else:
                rospy.loginfo("Waiting for odometry and path data...")
            self.reset_flags()
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

    def status_callback(self, msg):
        self.is_status = True
        self.ego = msg

    def stop(self):
        print("Logging finished")
        self.csv_file.close()

    def reset_flags(self):
        self.is_path = self.is_odom = self.is_status = False

if __name__ == '__main__':
    try:
        data_logger = DataLogger("tutorials", "log")
        data_logger.main()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            data_logger.stop()
        except NameError:
            pass
