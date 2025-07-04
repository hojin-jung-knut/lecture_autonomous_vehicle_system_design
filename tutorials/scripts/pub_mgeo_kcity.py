#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from lib.mgeo.class_defs import MGeoPlannerMap

class MGeoPublisher:
    def __init__(self):
        rospy.init_node('mgeo_pub', anonymous=True)

        self.link_pub = rospy.Publisher('/link', PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('/node', PointCloud, queue_size=1)

        # Use a ROS parameter for the MGeo data path
        mgeo_path = rospy.get_param("~mgeo_path", "lib/mgeo_data/kcity") # Default path
        load_path = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), mgeo_path))

        try:
            mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)
        except FileNotFoundError:
            rospy.logerr(f"MGeo data not found at: {load_path}")
            sys.exit(1)  # Exit with error code
        except Exception as e: # Catch other potential exceptions
            rospy.logerr(f"Error loading MGeo data: {e}")
            sys.exit(1)

        self.nodes = mgeo_planner_map.node_set.nodes
        self.links = mgeo_planner_map.link_set.lines

        self.link_msg = self.create_link_msg() # Create messages once
        self.node_msg = self.create_node_msg()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
            rospy.loginfo_throttle(3, "MGeo data published.") # Inform every 3 seconds
            rate.sleep()

    def create_link_msg(self):
        link_msg = PointCloud()
        link_msg.header.frame_id = 'map'
        for link_idx in self.links:
            for link_point in self.links[link_idx].points:
                point = Point32()
                point.x, point.y, point.z = link_point
                link_msg.points.append(point)
        return link_msg

    def create_node_msg(self):
        node_msg = PointCloud()
        node_msg.header.frame_id = 'map'
        for node_idx in self.nodes:
            point = Point32()
            point.x, point.y, point.z = self.nodes[node_idx].point
            node_msg.points.append(point)
        return node_msg

if __name__ == '__main__':
    try:
        mgeo_publisher = MGeoPublisher()
        rospy.spin() # Keep the node running
    except rospy.ROSInterruptException:
        pass
