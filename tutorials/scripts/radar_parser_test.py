#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, numpy as np
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, Float32MultiArray
from math import atan2, sqrt, pi

class RADARParser:
    def __init__(self):
        rospy.init_node('RADAR_parser', anonymous=True)
        self.radar_sub = rospy.Subscriber("/radarPointCloud", PointCloud, self.callback)
        self.radar_pub = rospy.Publisher('/sensor_radar', PointCloud, queue_size=1)
        self.dist_msg = Float32MultiArray()
        self.angle_msg = Float32MultiArray()

    def callback(self, msg):
        radar_out = PointCloud()
        radar_out.header = Header()
        radar_out.header.stamp = rospy.Time.now()
        radar_out.header.frame_id = "sensor_radar"
        radar_out.points = msg.points
        self.radar_pub.publish(radar_out)
        self.gmsg_np = self.geometrymsgs_to_dist(msg.points)
        self.dist_msg.data = self.calc_dist_forward()
        print("Distances:", self.dist_msg.data)
        print("Angles:", self.angle_msg.data)

        rospy.loginfo("Radar message published to '/sensor_radar'")
        rospy.loginfo("----------------------[ Radar_msg ]----------------------")
        for p in radar_out.points:
            rospy.loginfo(f"x: {p.x:.2f}, y: {p.y:.2f}, z: {p.z:.2f}")

    def geometrymsgs_to_dist(self, points):
        point_list = []
        for p in points:
            dist = sqrt(p.x**2 + p.y**2 + p.z**2)
            angle = atan2(p.y, p.x)*180/pi
            if p.x > 0 and dist < 100:
                point_list.append([dist, angle])
        return np.array(point_list, dtype=np.float32)

    def calc_dist_forward(self):
        angle_filter = (self.gmsg_np[:,1] > -60) & (self.gmsg_np[:,1] < 60)
        filtered_gmsg = self.gmsg_np[angle_filter]
        sorted_indices = np.argsort(filtered_gmsg[:,0])[:10]
        min_points = filtered_gmsg[sorted_indices]
        # print(sorted_indices)
        self.angle_msg.data = min_points[:,1]
        return min_points[:,0].tolist()

if __name__ == '__main__':
    try:
        RADARParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
