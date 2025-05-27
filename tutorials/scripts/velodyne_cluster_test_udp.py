#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy, numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float32MultiArray
from sklearn.cluster import DBSCAN
from math import atan2, sqrt, pi

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/sensor_lidar_clustering", PointCloud2, queue_size=10)
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
            xyz = np.mean(cluster[:,:3], axis=0)
            dist = np.min(cluster[:,4])
            angle = cluster[np.argmin(cluster[:,4]),5]
            self.cluster_points.append([xyz[0], xyz[1], xyz[2], dist, angle])
        d_min = self.calc_dist_forward()
        print(d_min)

        self.publish_point_cloud(self.cluster_points)
        if self.pc_np.size > 0:
            self.msg.data = list(map(float, self.cluster_points[0]))
            rospy.loginfo(f'''
----------------------[ Lidar_msg ]----------------------
[x(m), y(m), z(m), intensity, distance(m), angle(deg)] :
{self.msg.data}
                      ''')
        else:
            rospy.logwarn("Lidar data is empty after filtering")

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

        pc2_msg = pc2.create_cloud(header, fields, points)
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            self.dist = sqrt(point[0]**2 + point[1]**2)
            self.angle = atan2(point[1], point[0])*180/pi
            if point[0] > 0 and 1.50 > point[2] > -1.25 and self.dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], self.dist, self.angle))

        return np.array(point_list, np.float32)

    def calc_dist_forward(self):
        cluster_np = np.array(self.cluster_points)
        angle_filter = (cluster_np[:,4] > -60) & (cluster_np[:,4] < 60)
        filtered_cp = cluster_np[angle_filter]
        if filtered_cp.shape[0] == 0:
            return []
        sorted_indices = np.argsort(filtered_cp[:,3])[:10]
        min_points = filtered_cp[sorted_indices]
        return min_points[:,3].tolist()

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
