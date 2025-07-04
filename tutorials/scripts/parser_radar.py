#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, numpy as np
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from math import atan2, sqrt, pi

class RADARParser:
    def __init__(self):
        rospy.init_node('RADAR_parser', anonymous=True)
        rospy.Subscriber("/radarPointCloud", PointCloud, self.callback)
        self.radar_pub = rospy.Publisher('/sensor_radar', Float32MultiArray, queue_size=1)
        self.msg = Float32MultiArray()

    def callback(self, msg):
        self.gmsg_np = self.geometrymsgs_to_dist(msg.points)
        radar_data = self.calc_dist_angle()
        self.msg.data = radar_data.flatten().tolist()
        dim1 = MultiArrayDimension()
        dim1.label = "rows"
        dim1.size = radar_data.shape[0]
        dim1.stride = radar_data.size

        dim2 = MultiArrayDimension()
        dim2.label = "cols"
        dim2.size = radar_data.shape[1]
        dim2.stride = radar_data.shape[1]

        self.msg.layout.dim = [dim1, dim2]
        self.msg.layout.data_offset = 0        
        self.radar_pub.publish(self.msg)

        rospy.loginfo("Radar message published to '/sensor_radar'")
        rospy.loginfo(f'''
----------------------[ Radar_msg ]----------------------
{radar_data}''')

    def geometrymsgs_to_dist(self, points):
        point_list = []
        for p in points:
            dist = sqrt(p.x**2 + p.y**2)
            angle = atan2(p.y, p.x)*180/pi
            if p.x > 0 and dist < 100:
                point_list.append([dist, angle])
        return np.array(point_list, dtype=np.float32)

    def calc_dist_angle(self):
        if self.gmsg_np.size == 0 or self.gmsg_np.shape[1] < 2:
            return []
        angle_filter = (self.gmsg_np[:,1] > -60) & (self.gmsg_np[:,1] < 60)
        filtered_gmsg = self.gmsg_np[angle_filter]
        sorted_indices = np.argsort(filtered_gmsg[:,0])[:10]
        self.min_points = filtered_gmsg[sorted_indices]
        return self.min_points

if __name__ == '__main__':
    try:
        RADARParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
