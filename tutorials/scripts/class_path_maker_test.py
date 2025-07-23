#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg
from math import sqrt
from pyproj import Proj
from morai_msgs.msg import GPSMessage

class pathMaker:
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.is_gps_data = False
        self.x = 0; self.prev_x = 0
        self.y = 0; self.prev_y = 0     
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/' + path_name +'.txt'
        self.f = open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_gps_data == True :
                self.path_make()
        self.f.close()

    def path_make(self):
        x = self.x
        y = self.y
        distance = sqrt((x-self.prev_x)**2 + (y-self.prev_y)**2)
        if distance > 0.3:
            data='{0}\t{1}\n'.format(x,y)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            print("write : ", x,y)
    
    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        self.x = utm_x - gps_msg.eastOffset
        self.y = utm_y - gps_msg.northOffset

if __name__ == '__main__' :
    try:
        p_m = pathMaker("tutorials", "track_path")
    except rospy.ROSInternalException:
        pass
