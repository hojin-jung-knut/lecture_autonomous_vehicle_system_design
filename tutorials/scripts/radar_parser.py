#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, os, numpy as np
from std_msgs.msg import PointCloud
from morai_msgs.msg import RadarDetections
from math import pi

class RADARParser:
    def __init__(self):
        rospy.init_node('RADAR_parser', anonymous=True)
        self.radar_sub = rospy.Subscriber("/radarPointCloud", PointCloud, self.radar_callback)
        self.radar_pub = rospy.Publisher('/sensor_radar', Float32MultiArray, queue_size=10)
        # Initialization
        self.radar_msg = Float32MultiArray()
        self.radar_msg.channel = np.full([1,180],np.inf)
        self.radar_msg.point = np.full([1,180],np.inf)
        self.is_radar = False

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_radar == True :
                self.radar_pub.publish(self.radar_msg)
                print(f"radar_msg is now being published at '/sensor_radar' topic!\n")
                print('-----------------[ radar_msg ]---------------------')
                print(self.channel)

            if not self.is_radar:
                print("[1] Can't subscribe '/radar' topic... \n    please check your RADAR sensor connection")
            
            self.is_radar = False
            rate.sleep()

    def radar_callback(self, radar_msg):
        for i in radar_msg.channels :
            self.radar_msg.channel[i] = radar_msg.channel[i]
            self.radar_msg.point[i] = radar_msg.point[i]
        self.is_radar = True

if __name__ == '__main__':
    try:
        RADAR_parser = RADARParser()
    except rospy.ROSInterruptException:
        pass