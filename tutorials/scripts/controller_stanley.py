#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, numpy as np
import path_radius as path_radius
from math import cos, sin, sqrt, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum

class Stanley:
    def __init__(self):
        rospy.init_node('stanley', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)

        self.is_path = self.is_odom = self.is_status = False
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1
        self.target_vel = 50.0 # km/h
        self.vel = 0.0
        self.pose = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 3.0
        self.k, self.v_t = 1, 0.1
        self.accel_brake = PIDControl()
        self.radius = path_radius.Radius()
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.send_gear_cmd(Gear.D.value)

        self.rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                path_count = len(np.array(self.path.poses))
                if path_count < 5:
                    rospy.loginfo("Autonomous Driving Finished.")
                    self.send_gear_cmd(Gear.P.value)
                    break
                self.target_vel = 40.0
                # rospy.loginfo("radius: %d m, vx: %d km/h, vx_d: %d km/h", radius, self.vel*3.6, self.target_vel)
                self.stanley()
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            #self.reset_flags()
            self.rate.sleep()

    def stanley(self):
        self.is_look_forward_point = False
        temp = np.zeros((2, 2))
        front_axle_pos_x = self.pose.x + self.vehicle_length*cos(self.yaw)*0.5
        front_axle_pos_y = self.pose.y + self.vehicle_length*sin(self.yaw)*0.5
        min_dist, j = float('inf'), 0

        for num, p in enumerate(self.path.poses):
            dist = sqrt((p.pose.position.x - front_axle_pos_x)**2 + (p.pose.position.y - front_axle_pos_y)**2)
            if dist <= min_dist:
                min_dist = dist
                j = num
                temp[0][0], temp[0][1] = p.pose.position.x, p.pose.position.y
                self.is_look_forward_point = True
        if j + 1 < len(self.path.poses):
            temp[1][0] = self.path.poses[j+1].pose.position.x
            temp[1][1] = self.path.poses[j+1].pose.position.y
        else:
            temp[1] = temp[0]
        if self.is_look_forward_point:
            cte = sqrt((temp[0][0] - front_axle_pos_x)**2 + (temp[0][1] - front_axle_pos_y)**2)
            path_yaw = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
            heading_error = path_yaw - self.yaw
            if (temp[0][0] - front_axle_pos_x)*cos(path_yaw + pi/2) + (temp[0][1] - front_axle_pos_y)*sin(path_yaw + pi/2) < 0:
                cte *= -1.0
            while (heading_error > pi) : heading_error -= 2*pi
            while (heading_error < -pi) : heading_error += 2*pi

            crosstrack_error = atan2(self.k * cte, self.vx)
            self.ctrl_cmd_msg.steering = heading_error + crosstrack_error
            output = self.accel_brake.pid(self.target_vel, self.vel * 3.6)
            self.ctrl_cmd_msg.accel = min(1, max(output, 0))
            self.ctrl_cmd_msg.brake = min(1, max(-output, 0))
            rospy.loginfo(f"Accel: {self.ctrl_cmd_msg.accel*100:.2f} %, Brake: {self.ctrl_cmd_msg.brake*100:.2f} %, Wheel_Angle: {self.ctrl_cmd_msg.steering*180/pi:.2f} deg")
        else:
            rospy.logwarn("No forward point found")
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.accel = 0.0
            self.ctrl_cmd_msg.brake = 1

    def missing_topics(self):
        if not self.is_path:
            rospy.loginfo("Missing '/local_path' topic")
        if not self.is_odom:
            rospy.loginfo("Missing '/odom' topic")
        if not self.is_status:
            rospy.loginfo("Missing '/Ego_topic' topic")

    def reset_flags(self):
        self.is_path = self.is_odom = self.is_status = False

    def status_callback(self, msg):
        self.is_status = True
        # self.yaw = msg.heading*pi/180
        self.vel = msg.velocity.x
        self.vx = max(1e-1, self.vel)

    def local_path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def global_path_callback(self, msg):
        self.global_path = msg         

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.yaw = euler_from_quaternion(odom_quaternion)
        self.pose = msg.pose.pose.position

    def send_gear_cmd(self, gear_mode):
        while(abs(self.vel) > 0.1):
            self.ctrl_cmd_msg.brake = 1
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            self.rate.sleep()
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        self.event_cmd_srv(gear_cmd)


class PIDControl:
    def __init__(self, p_gain=0.1, i_gain=0.001, d_gain=0.003):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error, self.i_control = 0, 0
        self.anti_windup = 100.0
        self.t = time.time()

    def pid(self, target_vel, current_vel):
        t = time.time()
        dt = max(t - self.t, 1e-2)
        # rospy.loginfo(f"dt: {dt:.3f}")
        self.t = t
        error = target_vel - current_vel
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * dt
        self.i_control = max(min(self.i_control, self.anti_windup), -self.anti_windup)
        d_control = self.d_gain * (error - self.prev_error) / dt
        self.prev_error = error
        return p_control + self.i_control + d_control


class Gear(Enum):
	P, R, N, D = 1, 2, 3, 4


if __name__ == '__main__':
    try:
        Stanley()
    except rospy.ROSInterruptException:
        pass
