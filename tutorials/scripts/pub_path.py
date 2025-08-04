#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, time
from math import sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class PubPath:
    def __init__(self):
        rospy.init_node('pub_path', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.is_odom = False
        self.is_loop = False
        self.start_time = time.time()

        full_global_path, global_path = [], []
        local_path_size = 50
        block = 0
        k, k_init = 0, 0
        current_idx_init = 0
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('tutorials')
        #full_path = pkg_path + '/path/k-city.txt'
        full_path = pkg_path+'/path/c-track.txt'

        with open(full_path, 'r') as f:
            for line in f.readlines():
                tmp = line.strip().split()
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(tmp[0])
                pose.pose.position.y = float(tmp[1])
                pose.pose.orientation.w = 1
                full_global_path.append(pose)
        global_path = full_global_path
        block = len(full_global_path)//30+1

        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if self.is_odom:
                x, y = self.x, self.y
                min_dis = float('inf')
                current_idx = -1
                for i, pose in enumerate(global_path):
                    dx = x - pose.pose.position.x
                    dy = y - pose.pose.position.y
                    dist = sqrt(dx**2 + dy**2)
                    if dist < min_dis:
                        min_dis = dist
                        current_idx = i
                if k_init < 1:
                    k = current_idx//block 
                    if k < 3:
                        if self.is_loop:
                            global_path = full_global_path[(27+k)*block:] + full_global_path[:(k+3)*block]
                            current_idx_init = len(global_path)//2
                        else:
                            global_path = full_global_path[:(k+3)*block]
                            current_idx_init = k*block
                    elif k > 27:
                        if self.is_loop:
                            global_path = full_global_path[(k-3)*block:] + full_global_path[:(k-27)*block]
                            current_idx_init = len(global_path)//2
                        else:
                            global_path = full_global_path[(k-3)*block:]
                            current_idx_init = 3*block
                    else:
                        global_path = full_global_path[(k-3)*block:(k+3)*block]
                        current_idx_init = len(global_path)//2
                    k_init = 1
                print(f"k: {k}, current_idx: {current_idx}, current_idx_init: {current_idx_init}")
                if (current_idx - current_idx_init) > block:
                    k += 1
                    k %= 30
                    if k < 3:
                        if self.is_loop:
                            global_path = full_global_path[(27+k)*block:] + full_global_path[:(k+3)*block]
                            current_idx_init = len(global_path)//2
                        else:
                            global_path = full_global_path[:(k+3)*block]
                            current_idx_init = k*block
                    elif k > 27:
                        if self.is_loop:
                            global_path = full_global_path[(k-3)*block:] + full_global_path[:(k-27)*block]
                            current_idx_init = len(global_path)//2
                        else:
                            global_path = full_global_path[(k-3)*block:]
                            current_idx_init = 3*block
                    else:
                        global_path = full_global_path[(k-3)*block:(k+3)*block]
                        current_idx_init = len(global_path)//2
                    continue
                if current_idx == -1:
                    rospy.logwarn("Failed to find nearest waypoint.")
                    rate.sleep()
                    continue

                # Global path
                global_path_msg = Path()
                global_path_msg.header.frame_id = 'map'
                global_path_msg.poses = global_path

                # Local path
                local_end = min(len(global_path), current_idx + local_path_size)
                local_path_msg = Path()
                local_path_msg.header.frame_id = 'map'
                local_path_msg.poses = global_path[current_idx:local_end]

                #if time.time() - self.start_time < 5:
                self.global_path_pub.publish(global_path_msg)
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        PubPath()
    except rospy.ROSInterruptException:
        pass
