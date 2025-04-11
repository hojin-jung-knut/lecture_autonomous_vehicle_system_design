#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, numpy as np
from math import cos, sin, sqrt, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf.transformations import euler_from_quaternion

class PIDControl:
    def __init__(self, p_gain=0.3, i_gain=0.07, d_gain=0.03, control_time=0.02):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.control_time = control_time
        self.prev_error = self.i_control = 0

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error

        if abs(error) <= 5:
            self.i_control += self.i_gain * error * self.control_time

        d_control = self.d_gain * (error - self.prev_error) / self.control_time
        self.prev_error = error

        return p_control + self.i_control + d_control

class Stanley:
    def __init__(self):
        rospy.init_node('stanley', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        self.is_path = self.is_odom = self.is_status = False
        self.current_position = Point()
        self.target_vel = rospy.get_param('~target_velocity', 30.0)
        self.vehicle_length = rospy.get_param('~vehicle_length', 3.0)
        self.k = rospy.get_param('~stanley_gain', 0.8)
        self.v_t = rospy.get_param('~velocity_gain', 1.0)

        self.pid_controller = PIDControl()
        self.control_loop()
    
    def control_loop(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.stanley_control()
            else:
                self.missing_topics()

            self.reset_flags()
            rate.sleep()

    def stanley_control(self):
        vehicle_position = self.current_position
        self.is_look_forward_point = False
        translation_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), vehicle_position.x],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), vehicle_position.y],
            [0, 0, 1]
        ])
        inv_translation_matrix = np.linalg.inv(translation_matrix)

        temp = np.zeros((2, 2))  # for saving local point of the distant minimum path point
        dis_min = float('inf')
        j = 0

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = inv_translation_matrix.dot(global_path_point)
            if local_path_point[0] < 0:
                continue
            dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
            if dis <= dis_min:
                dis_min = dis
                j = num
                temp[0][0] = local_path_point[0]
                temp[0][1] = local_path_point[1]
                temp_global = global_path_point
                self.is_look_forward_point = True
            if num == j + 1:
                temp[1][0] = local_path_point[0]
                temp[1][1] = local_path_point[1]

        if self.is_look_forward_point:
            self.publish_control_commands(temp, temp_global, vehicle_position)
        else:
            self.ctrl_cmd_msg.steering = self.ctrl_cmd_msg.accel = self.ctrl_cmd_msg.brake = 0.0
            rospy.logwarn("Forward point not found")

    def publish_control_commands(self, temp, temp_global, vehicle_position):
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1 # Acceleration control type
        
        heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
        cte = sin(self.vehicle_yaw) * (temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw) * (temp_global[1] - vehicle_position.y)
        crosstrack_error = -atan2(self.k * cte, self.current_vel + self.v_t)

        steering_angle = heading_error + crosstrack_error
        steering_angle = np.clip(steering_angle, -pi/6, pi/6)
        self.ctrl_cmd_msg.steering = steering_angle
        output = self.pid_controller.pid(self.target_vel, self.current_vel * 3.6)
        self.ctrl_cmd_msg.accel = max(0, output)
        self.ctrl_cmd_msg.brake = max(0, -output)
        rospy.loginfo(f"Steering (deg): {self.ctrl_cmd_msg.steering * 180 / pi:.2f}, Accel: {min(100,self.ctrl_cmd_msg.accel):.2f}, Brake: {min(100,self.ctrl_cmd_msg.brake):.2f}")
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def missing_topics(self):
        if not self.is_path:
            rospy.logwarn("Missing '/local_path' topic")
        if not self.is_odom:
            rospy.logwarn("Missing '/odom' topic")
        if not self.is_status:
            rospy.logwarn("Missing '/Ego_topic' topic")

    def reset_flags(self):
        self.is_path = self.is_odom = self.is_status = False  

    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position = msg.pose.pose.position

if __name__ == '__main__':
    try:
        stanley = Stanley()
    except rospy.ROSInterruptException:
        pass
