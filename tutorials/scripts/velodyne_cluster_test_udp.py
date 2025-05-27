#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy, numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from math import atan2, sqrt

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/sensor_lidar_clustering", PointCloud2, queue_size=10)
        self.pc_np = None
        eps = rospy.get_param("~eps", 0.5)
        min_samples = rospy.get_param("~min_samples", 5)
        self.dbscan = DBSCAN(eps=eps, min_samples=min_samples)

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return

        pc_xy = self.pc_np[:, [0, 1]]
        db = self.dbscan.fit_predict(pc_xy)
        # valid_indices = db != -1
        # labels = db[valid_indices]
        # pc_xy = pc_xy[valid_indices]
        n_cluster = np.max(db) + 1

        cluster_points = []
        for c in range(n_cluster):
            c_tmp = np.mean(pc_xy[db==c, :], axis=0)
            z_mean = np.mean(self.pc_np[db==c, 2])
            cluster_points.append([c_tmp[0], c_tmp[1], z_mean])

        self.publish_point_cloud(cluster_points)

    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('dist', 12, PointField.FLOAT32, 1),
            PointField('angle', 16, PointField.FLOAT32, 1),
        ]

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, points)

        # Publish PointCloud2 message
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            self.dist = sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            self.angle = atan2(point[1], point[0])
            if point[0] > 0 and 1.50 > point[2] > -1.25 and self.dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], self.dist, self.angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin() 
