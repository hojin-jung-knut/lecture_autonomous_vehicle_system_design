#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np
from math import cos, sin, sqrt, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from tf.transformations import euler_from_quaternion
import path_curvature
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum

class Gear(Enum):
	P = 1
	R = 2
	N = 3
	D = 4


class PIDControl:
    def __init__(self, p_gain=0.3, i_gain=0.03, d_gain=0.003, control_time=0.01):
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

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)   
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1) # check topic in MORAI Sim (/ctrl_cmd_0)
        
        self.is_path = self.is_odom = self.is_status = False
        self.target_vel = 60.0  # in m/s
        self.current_vel = 0.0
        self.forward_point = self.current_position = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 3.0
        self.lfd = 5.0 # Look-ahead distance
        self.pid_controller = PIDControl()
        self.curv = path_curvature.Curvature()
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.send_gear_cmd(Gear.D.value)
        self.control_loop()

    def control_loop(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                path_count = len(np.array(self.path.poses)) -5
                # if path_count < 5: self.target_vel = 0
                if path_count > 5:
                    curvature = self.curv.path_curvature(self.path, self.current_position, path_count)
                print(path_count)
                if path_count < 7 :
                    print("DONE!")
                    self.send_gear_cmd(Gear.P.value)
                    break
                if curvature <= 200:
                # print(sqrt(curvature * 9.81 * 1))
                    self.target_vel = sqrt(curvature * 9.81 * 1)
                    if 50 < curvature < 200:
                        self.target_vel = 40.0
                    if curvature < 40 :
                        self.target_vel  = 10.0
                else: self.target_vel = 60.0
                print("curvature / self.target_vel", curvature, self.target_vel)
                rospy.logwarn("curvature :%d", curvature)
                self.pure_pursuit_control()
            else:
                self.missing_topics()
            self.reset_flags()
            self.rate.sleep()

    def pure_pursuit_control(self):
        translation = [self.current_position.x, self.current_position.y]
        transformation_matrix = self.get_transformation_matrix(translation)
        inv_transformation_matrix = np.linalg.inv(transformation_matrix)

        self.is_look_forward_point = False
        for pose in self.path.poses:
            local_path_point = self.transform_point(inv_transformation_matrix, pose.pose.position)

            if local_path_point[0] > 0:
                distance = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                if distance >= self.lfd:
                    self.forward_point = pose.pose.position
                    self.is_look_forward_point = True
                    break

        if self.is_look_forward_point:
            self.publish_control_commands(local_path_point)
        else:
            rospy.logwarn("No forward point found")
            self.publish_control_commands([0, 0, 0], err=True)

    def publish_control_commands(self, local_path_point, err=False):
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1 # Acceleration control type

        if err:
            self.ctrl_cmd_msg.steering = self.ctrl_cmd_msg.velocity = 0.0
        else:
            theta = atan2(local_path_point[1], local_path_point[0])
            self.lfd = self.current_vel * 0.7
            if self.lfd < 5: self.lfd = 5
            self.ctrl_cmd_msg.steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)
            output = self.pid_controller.pid(self.target_vel, self.current_vel * 3.6)

            self.ctrl_cmd_msg.accel = max(0, output)
            self.ctrl_cmd_msg.brake = max(0, -output)

            rospy.loginfo(f"Accel: {min(100,self.ctrl_cmd_msg.accel):.2f}%, Brake: {min(100,self.ctrl_cmd_msg.brake):.2f}%")

        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def transform_point(self, transformation_matrix, point):
        global_path_point = np.array([point.x, point.y, 1])
        local_path_point = transformation_matrix.dot(global_path_point)
        return local_path_point

    def get_transformation_matrix(self, translation):
        return np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

    def reset_flags(self):
        self.is_path = self.is_odom = self.is_status = False

    def missing_topics(self):
        if not self.is_path:
            rospy.logwarn("Missing '/local_path' topic")
        if not self.is_odom:
            rospy.logwarn("Missing '/odom' topic")
        if not self.is_status:
            rospy.logwarn("Missing '/Ego_topic' topic")

    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position = msg.pose.pose.position

    def send_gear_cmd(self, gear_mode):
		# 기어 변경이 제대로 되기 위해서는 차량 속도가 약 0이어야 함.
        while(abs(self.current_vel) > 0.1):
            self.ctrl_cmd_msg.brake = 1
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            self.rate.sleep()
            
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        # gear_cmd.resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)    
        
if __name__ == '__main__':
    try:
        pure_pursuit = PurePursuit()
    except rospy.ROSInterruptException:
        pass
