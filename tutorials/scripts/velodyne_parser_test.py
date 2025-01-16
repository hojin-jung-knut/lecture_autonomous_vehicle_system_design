#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32
from math import atan2, sqrt, pi

class SCANParser:
    def __init__(self):
        rospy.init_node('velodyne_parser', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.dist_pub = rospy.Publisher("/dist_forward", Float32, queue_size=10)
        self.pc_np = None

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if self.pc_np.size > 0:
            d_min = self.calc_dist_forward()
            dist_msg = Float32()
            dist_msg.data = d_min
            self.dist_pub.publish(dist_msg)
            rospy.loginfo(f'''
----------------------[ LiDAR data ]----------------------
                Distance : {dist_msg.data}
                      ''')
        else:
            rospy.logwarn("LiDAR data is empty after filtering")

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = atan2(point[1], point[0])
            if point[0] > 0 and point[2] > -1.3 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))
        return np.array(point_list, np.float32)

    def calc_dist_forward(self):
        r1_bool = self.pc_np[:, 5] > -30/180*pi
        r2_bool = self.pc_np[:, 5] < 30/180*pi
        d1 = self.pc_np[r1_bool, 4]
        d2 = self.pc_np[r2_bool, 4]
        d_list = np.concatenate([d1, d2])
        return np.min(d_list)

if __name__ == '__main__':
    try:
        SCANParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("SCANParser terminated.")