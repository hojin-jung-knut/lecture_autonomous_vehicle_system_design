#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
from math import atan2, sqrt, pi

class SCANParser:
    def __init__(self):
        rospy.init_node('velodyne_parser', anonymous=True)
        self.scan_sub = rospy.Subscriber("/lidar3D", PointCloud2, self.callback)
        self.dist_pub = rospy.Publisher("/sensor_lidar", Float32MultiArray, queue_size=10)
        self.pc_np = None
        self.msg = Float32MultiArray()
        self.dist_msg = Float32MultiArray()
        self.angle_msg = Float32MultiArray()

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if self.pc_np.size > 0:
            self.msg.data = [self.pc_np[0][0], self.pc_np[0][1], self.pc_np[0][2], self.pc_np[0][3], self.pc_np[0][4], self.pc_np[0][5]]
            # print(self.pc_np.size)
            # print(len(self.pc_np[:,4]))
            self.dist_msg.data = self.calc_dist_forward()
            print(self.dist_msg.data)
            print(self.angle_msg.data)
            self.dist_pub.publish(self.dist_msg)
            rospy.loginfo(f'''
----------------------[ Lidar_msg ]----------------------
[x(m), y(m), z(m), intensity, distance(m), angle(deg)] :
{self.msg.data}
                      ''')
        else:
            rospy.logwarn("Lidar data is empty after filtering")

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = atan2(point[1], point[0])*180/pi
            if point[0] > 0 and point[2] > -1.3 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle)) #x, y, z, intensity, dist, angle
        return np.array(point_list, np.float32)

    def calc_dist_forward(self):
        angle_filter = (self.pc_np[:,5] > -60) & (self.pc_np[:,5] < 60)
        filtered_pc = self.pc_np[angle_filter]
        sorted_indices = np.argsort(filtered_pc[:,4])[:10]
        min_points = filtered_pc[sorted_indices]
        print(sorted_indices)
        self.angle_msg.data = min_points[:,5]
        return min_points[:, 4].tolist()

if __name__ == '__main__':
    try:
        SCANParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("SCANParser terminated.")
