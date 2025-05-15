#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, std_msgs.msg
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class RADARParser:
    def __init__(self):
        rospy.init_node('RADAR_parser', anonymous=True)
        self.radar_sub = rospy.Subscriber("/radarPointCloud", PointCloud, self.callback)
        self.radar_pub = rospy.Publisher('/sensor_radar', PointCloud, queue_size=1)
        # Initialization
        self.header = std_msgs.msg.Header()
        self.header.stamp = rospy.Time.now()
        self.header.frame_id = '/sensor_radar'
        self.radar_msg = PointCloud()
        self.radar_points = [Point32() for _ in range(180)]
        self.is_radar = False
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.is_radar :
                self.radar_msg.header = self.header
                self.radar_msg.points = self.radar_points
                self.radar_pub.publish(self.radar_msg)
                rospy.loginfo("Radar message published to '/sensor_radar'")
                rospy.loginfo("----------------------[ radar_msg ]----------------------")
                rospy.loginfo(self.radar_msg.points)
            else :
                rospy.logwarn("Unable to subscribe to '/radarPointCloud'. Check RADAR sensor connection.")
            self.is_radar = False
            rate.sleep()

    def callback(self, radar_msg):
        for i in range(len(radar_msg.points)) :
            self.radar_points[i] = Point32(
                x = radar_msg.points[i].x,
                y = radar_msg.points[i].y,
                z = radar_msg.points[i].z
            )
        self.is_radar = True

if __name__ == '__main__':
    try:
        RADARParser()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
