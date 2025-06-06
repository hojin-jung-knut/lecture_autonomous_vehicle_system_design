#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, numpy as np
from math import cos, sin, atan2, sqrt, pi
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/sensor_radar", Float32MultiArray, self.radar_callback)
        rospy.Subscriber("/sensor_lidar", Float32MultiArray, self.lidar_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size = 1)
        self.is_path = self.is_radar = self.is_lidar = self.is_odom = False
        self.xprev = 0
        self.tprev = time.time()
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_lidar and self.is_odom:
                if self.checkObject(self.local_path, self.odom.pose.pose):
                    lattice_path = self.latticePlanner(self.local_path, self.odom.pose.pose)
                    lattice_path_index = self.collision_check(lattice_path, self.odom.pose.pose)
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, vpos):
        is_crash = False
        for i in range(self.lidar_data.shape[0]):
            for path in ref_path.poses:
                angle = euler_from_quaternion([vpos.orientation.x,vpos.orientation.y,vpos.orientation.z,vpos.orientation.w])
                dis = sqrt((path.pose.position.x - (vpos.position.x+self.lidar_data[i,0]*cos(self.lidar_data[i,1]*pi/180 + angle[2])))**2 + 
                           (path.pose.position.y - (vpos.position.y+self.lidar_data[i,0]*sin(self.lidar_data[i,1]*pi/180 + angle[2])))**2)
                if dis < 2.35:          # A collision is considered to occur when the straight-line distance between 
                    is_crash = True     # the obstacle's coordinates and the coordinates on the local path is less than 2.35
                    break
        return is_crash

    def collision_check(self, out_path, vpos):
        #TODO: (6) Select the lowest cost path among the generated collision avoidance paths
        
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path
        for i in range(self.lidar_data.shape[0]):                     
            for path_num in range(len(out_path)):                    
                for path_pos in out_path[path_num].poses:
                    angle = euler_from_quaternion([vpos.orientation.x,vpos.orientation.y,vpos.orientation.z,vpos.orientation.w])
                    dis = sqrt((path_pos.pose.position.x - (vpos.position.x+self.lidar_data[i,0]*cos(self.lidar_data[i,1]*pi/180 + angle[2])))**2 + 
                           (path_pos.pose.position.y - (vpos.position.y+self.lidar_data[i,0]*sin(self.lidar_data[i,1]*pi/180 + angle[2])))**2)
                    if dis < 10:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))     

        return selected_lane

    def radar_callback(self,msg):
        self.is_radar = True
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.radar_data = np.array(msg.data, dtype=np.float32).reshape((rows, cols))
        #rospy.loginfo("Received array:\n%s", data)

    def lidar_callback(self,msg):
        self.is_lidar = True
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.lidar_data = np.array(msg.data, dtype=np.float32).reshape((rows, cols))
        #print(self.lidar_data[0,1])

    def odom_callback(self,msg):
        self.is_odom = True
        self.odom = msg  

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg

    def latticePlanner(self, ref_path, vpos):
        out_path = []
        vehicle_pose_x = vpos.position.x
        vehicle_pose_y = vpos.position.y   
        vehicle_velocity = (vpos.position.x - self.xprev) / (time.time()-self.tprev) * 3.6
        self.xprev = vpos.position.x
        self.tprev = time.time()
        look_distance = int(vehicle_velocity * 0.2 * 2)
        
        if look_distance < 20 :
            look_distance = 20                    

        if len(ref_path.poses) > look_distance :  
            #TODO: (3) Generate coordinate transformation matrix
        
            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice collision avoidance path generation
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf**2)
                a[3] = -2.0 * (pf - ps) / (xf**3)
                
                # 3rd order function graph
                for i in x:
                    result = a[3] * i**3 + a[2] * i**2 + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)
            
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2, add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i+1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i+1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
                        
            #TODO: (5) Publish all generated Lattice collision avoidance path messages
            self.lattice_pubs = [rospy.Publisher(f'/lattice_path_{i+1}', Path, queue_size=1) for i in range(6)]
            for i in range(len(out_path)):
                self.lattice_pubs[i].publish(out_path[i])           
        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
