#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, numpy as np
from math import cos, sin, sqrt, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from enum import Enum
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv

class Stanley:
    def __init__(self):
        rospy.init_node('stanley', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)

        self.is_path = self.is_odom = self.is_status = False
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.current_position = Point()
        self.target_vel = 30.0 # km/h
        self.current_vel = 0.0
        self.k = 0.8
        self.v_t = 1
        self.accel_brake = PIDControl()

        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.send_gear_cmd(Gear.D.value)
        self.control_loop()

    def control_loop(self):
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.stanley()
            else:
                self.missing_topics()
            self.reset_topic_flags()
            rate.sleep()

    def stanley(self):
        vehicle_position = self.current_position
        self.is_look_forward_point = False
        t = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), vehicle_position.x],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), vehicle_position.y],
            [0, 0, 1]
        ])
        inv_t = np.linalg.inv(t)
        temp = np.zeros((2, 2)) # for saving local point of the distant minimum path point
        dis_min = float('inf')
        j = 0

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = inv_t.dot(global_path_point)
            if local_path_point[0] < 0:
                continue
            dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
            if dis <= dis_min:
                dis_min = dis
                j = num
                temp[0][0], temp[0][1] = local_path_point[0], local_path_point[1] 
                temp_global = global_path_point
                self.is_look_forward_point = True
            if num == j + 1:
                temp[1][0], temp[1][1] = local_path_point[0], local_path_point[1] 

        if self.is_look_forward_point:
            heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
            cte = sin(self.vehicle_yaw) * (temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw) * (temp_global[1] - vehicle_position.y)
            crosstrack_error = -atan2(self.k * cte, self.current_vel + self.v_t)
            self.ctrl_cmd_msg.steering = heading_error + crosstrack_error
            output = self.accel_brake.pid(self.target_vel, self.current_vel * 3.6)
            self.ctrl_cmd_msg.accel = min(1, max(output, 0))
            self.ctrl_cmd_msg.brake = min(1, max(-output, 0))
            rospy.loginfo(f"Accel: {self.ctrl_cmd_msg.accel*100:.2f} %, Brake: {self.ctrl_cmd_msg.brake*100:.2f} %, Wheel_Angle: {self.ctrl_cmd_msg.steering*180/pi:.2f} deg")
        else:
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.accel = 0.0
            self.ctrl_cmd_msg.brake = 1
            rospy.logwarn("Forward point not found")

        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def missing_topics(self):
        if not self.is_path:
            rospy.loginfo("Missing '/local_path' topic")
        if not self.is_odom:
            rospy.loginfo("Missing '/odom' topic")
        if not self.is_status:
            rospy.loginfo("Missing '/Ego_topic' topic")

    def reset_topic_flags(self):
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

    def send_gear_cmd(self, gear_mode):
		# Vehicle velocity should be zero for gear change
        while(abs(self.current_vel) > 0.1):
            self.ctrl_cmd_msg.brake = 1
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            self.rate.sleep()
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        self.event_cmd_srv(gear_cmd)


class PIDControl:
    def __init__(self, p_gain=0.1, i_gain=0.001, d_gain=0.003, control_time=0.01):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.control_time = control_time
        self.prev_error, self.i_control = 0, 0
        self.t = 0

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        if abs(error) <= 5: self.i_control += self.i_gain * error * self.control_time
        d_control = self.d_gain * (error - self.prev_error) / self.control_time
        self.prev_error = error
        t = self.t
        self.t = time.time()
        self.control_time = min(1, self.t - t)
        return p_control + self.i_control + d_control


class Gear(Enum):
	P, R, N, D = 1, 2, 3, 4


if __name__ == '__main__':
    try:
        Stanley()
    except rospy.ROSInterruptException:
        pass
