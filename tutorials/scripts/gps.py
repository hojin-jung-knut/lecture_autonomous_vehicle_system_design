#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
from pyproj import Proj
from morai_msgs.msg import GPSMessage

class GPS_to_UTM:
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.is_gps_data = False

    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        map_x = utm_x - gps_msg.eastOffset
        map_y = utm_y - gps_msg.northOffset

        os.system('clear')
        print(f'''
------------------------[ GPS data ]--------------------

                latitude : {latitude}
                longitude : {longitude}
                altitude : {altitude}
                            |
                            | apply Projection (utm 52 Zone)
                            V

----------------------[ utm ] ---------------------
                utm_x    : {utm_x}
                utm_y    : {utm_y}

                            |
                            | apply offset (east and north)
                            V
------------------- [ map ] -------------------
                simulator map_x : {map_x}
                simulator map_y : {map_y}
        ''')

if __name__ == '__main__':
    try:
        gps_to_utm = GPS_to_UTM()
        rospy.spin()  # Keeps the node from exiting
    except rospy.ROSInterruptException:
        pass