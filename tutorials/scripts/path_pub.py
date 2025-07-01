#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, time
from math import sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class path_pub:
    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)

        self.full_global_path, self.global_path = [], []
        self.local_path_size = 50
        self.k = 0
        self.k_init = 0
        self.is_odom = False
        self.start_time = time.time()

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('tutorials')
        full_path = pkg_path + '/path/k-city.txt'
        #full_path = pkg_path+'/path/c-track.txt'
        print(full_path)

        with open(full_path, 'r') as f:
            for line in f.readlines():
                tmp = line.strip().split()
                pose = PoseStamped()
                pose.pose.position.x = float(tmp[0])
                pose.pose.position.y = float(tmp[1])
                pose.pose.orientation.w = 1
                self.full_global_path.append(pose)
        self.global_path = self.full_global_path
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            if self.is_odom:
                x, y = self.x, self.y
                min_dis = float('inf')
                current_idx = -1
                for i, pose in enumerate(self.global_path):
                    dx = x - pose.pose.position.x
                    dy = y - pose.pose.position.y
                    dist = sqrt(dx**2 + dy**2)
                    if dist < min_dis:
                        min_dis = dist
                        current_idx = i
                        if self.k_init < 1:
                            self.k = int(current_idx/len(self.full_global_path)*30)
                            self.k_init = 1
                if current_idx == -1:
                    rate.sleep()
                    continue
                current_block = len(self.full_global_path)//30
                if current_idx > current_block:
                    self.k += 1
                    self.global_path = self.full_global_path[self.k*current_block:]
                if self.k == 29 and current_idx > len(self.global_path) - 5:
                    self.global_path = self.full_global_path
                    self.k = 0
                
                # Global path
                global_path_msg = Path()
                global_path_msg.header.frame_id = 'map'
                global_path_msg.poses = self.full_global_path

                # Local path
                local_end = min(len(self.global_path), current_idx + self.local_path_size)
                local_path_msg = Path()
                local_path_msg.header.frame_id = 'map'
                local_path_msg.poses = self.global_path[current_idx:local_end]

                if time.time() - self.start_time < 5:
                    self.global_path_pub.publish(global_path_msg)
                self.local_path_pub.publish(local_path_msg)

                print(f"Current position: ({x:.2f}, {y:.2f})")

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        path = path_pub()
    except rospy.ROSInterruptException:
        pass
