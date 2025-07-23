#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
import tf
from math import sqrt
from geometry_msgs.msg import PoseStamped
from path_reader import pathReader
from turtlesim.msg import Pose

def find_local_path(ref_path, status_msg):
    # Generate local path from the reference path and the current status of the turtle.
    out_path = Path()
    current_x = status_msg.x
    current_y = status_msg.y
    current_waypoint = 0
    min_dis = float('inf')

    # Find the closest waypoint to the current position
    for i in range(len(ref_path.poses)):
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dis = sqrt(dx**2 + dy**2)
        if dis < min_dis:
            min_dis = dis
            current_waypoint = i

    # Determine the range of waypoints to include in the local path
    last_local_waypoint = min(current_waypoint + 10, len(ref_path.poses))

    out_path.header.frame_id = 'map'

    # Populate the local path with waypoints
    for i in range(current_waypoint, last_local_waypoint):
        tmp_pose = PoseStamped()
        tmp_pose.pose.position.x = ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y = ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z = ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation = ref_path.poses[i].pose.orientation
        out_path.poses.append(tmp_pose)

    return out_path, current_waypoint

class TurtleListener:
    def __init__(self):
        rospy.Subscriber('/turtle1/pose', Pose, self.status_cb)
        self.status_msg = Pose()

    def status_cb(self, data):
        # Callback function to update the turtle's status.
        self.status_msg = data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.x, self.status_msg.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.status_msg.theta),
                         rospy.Time.now(),
                         "turtle",
                         "map")

if __name__ == '__main__':
    try:
        rospy.init_node('local_path_finder', anonymous=True)
        path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        tl = TurtleListener()

        # Load global path
        p_r = pathReader("tutorials")
        global_path = p_r.read_txt("turtle_path.txt")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Generate and publish local path
            local_path, current_waypoint = find_local_path(global_path, tl.status_msg)
            local_path_pub.publish(local_path)
            path_pub.publish(global_path)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
