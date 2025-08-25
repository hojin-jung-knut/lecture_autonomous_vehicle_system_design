#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, time, csv, os
import numpy as np
from nav_msgs.msg import Odometry, Path
from math import sqrt
from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus
from tf.transformations import euler_from_quaternion

class DataLogger:
    def __init__(self, pkg_name, path_name):
        rospy.init_node('data_logger', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.is_global_path = self.is_path = self.is_odom = self.is_status = False
        self.ref_point, self.pose = Point(), Point()
        self.log_start_time = time.time()
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        self.log_file_path = os.path.join(pkg_path, f"{path_name}.csv")
        self.csv_file = open(self.log_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'xd', 'y', 'yd', 'vx(km/h)', 'wheel_angle(deg)', 'accel', 'brake'])
        self.last_pose_x, self.last_pose_y = 0.0, 0.0

    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_global_path and self.is_path and self.is_odom and self.is_status:
                self.closest_points()
                log = np.around([time.time() - self.log_start_time, self.pose.x, self.ref_point.x, self.pose.y, 
                    self.ref_point.y, self.ego.velocity.x*3.6, self.ego.wheel_angle, self.ego.accel, self.ego.brake], 4)
                self.csv_writer.writerow(log)
                if len(np.array(self.path.poses)) < 5:
                    rospy.loginfo("Data Logging Finished.")
                    self.csv_file.close()
                    break
            else:
                rospy.loginfo("Waiting for odometry and path data...")
            
            #self.reset_flags()
            rate.sleep()
    
    def closest_points(self):
        min_dist = float('inf')
        for pose in self.global_path.poses:
            dist = sqrt((pose.pose.position.x - self.pose.x)**2 + (pose.pose.position.y - self.pose.y)**2)
            if dist <= min_dist:
                min_dist = dist
                self.ref_point = pose.pose.position

    def global_path_callback(self, msg):
        self.is_global_path = True
        self.global_path = msg

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

    def reset_flags(self):
        self.is_path = self.is_odom = self.is_status = False

    def stop(self):
        rospy.loginfo("Data Logging Finished.")
        self.csv_file.close()

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
