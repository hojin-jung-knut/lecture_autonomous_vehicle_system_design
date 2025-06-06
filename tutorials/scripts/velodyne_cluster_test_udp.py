#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy, numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sklearn.cluster import DBSCAN
from math import atan2, sqrt, pi

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.lidar_pub = rospy.Publisher("/sensor_lidar", Float32MultiArray, queue_size=10)
        self.pc_np = None
        eps = rospy.get_param("~eps", 0.5)
        min_samples = rospy.get_param("~min_samples", 5)
        self.dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        self.msg = Float32MultiArray()

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return
        pc_xy = self.pc_np[:,[0,1]]
        db = self.dbscan.fit_predict(pc_xy)
        n_cluster = np.max(db) + 1
        if n_cluster == 0:
            return
        self.cluster_points = []
        for c in range(n_cluster):
            cluster = self.pc_np[db == c]
            dist = np.min(cluster[:,0])
            angle = cluster[np.argmin(cluster[:,0]),1]
            intensity = np.mean(cluster[:,2])
            self.cluster_points.append([dist, angle, intensity])
        lidar_data = np.array(self.calc_dist_angle())
        if lidar_data.size == 0:
            return
        self.msg.data = lidar_data.flatten().tolist()
        dim1 = MultiArrayDimension()
        dim1.label = "rows"
        dim1.size = lidar_data.shape[0]
        dim1.stride = lidar_data.size

        dim2 = MultiArrayDimension()
        dim2.label = "cols"
        dim2.size = lidar_data.shape[1]
        dim2.stride = lidar_data.shape[1]

        self.msg.layout.dim = [dim1, dim2]
        self.msg.layout.data_offset = 0        
        self.lidar_pub.publish(self.msg)

        rospy.loginfo("Lidar message published to '/sensor_lidar'")
        rospy.loginfo(f'''
----------------------[ Lidar_msg ]----------------------
{lidar_data}''')

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = sqrt(point[0]**2 + point[1]**2)
            angle = atan2(point[1], point[0])*180/pi
            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((dist, angle, point[3]))
        return np.array(point_list, np.float32)

    def calc_dist_angle(self):
        cluster_np = np.array(self.cluster_points)
        if cluster_np.size == 0 or cluster_np.shape[1] < 3:
            return []
        angle_filter = (cluster_np[:,1] > -60) & (cluster_np[:,1] < 60)
        filtered_cp = cluster_np[angle_filter]
        if filtered_cp.shape[0] == 0:
            return []
        sorted_indices = np.argsort(filtered_cp[:,0])[:10]
        min_points = filtered_cp[sorted_indices]
        return min_points

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
