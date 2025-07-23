#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum

class CollisionAvoid():
	def __init__(self):
		rospy.init_node('collision_avoid', anonymous=True)
		rospy.Subscriber('/CollisionData', CollisionData, self.collision_callback)
		rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
		self.cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size = 1)
		rospy.wait_for_service('/Service_MoraiEventCmd')
		self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
		self.is_collision = False
		self.ego_status = EgoVehicleStatus()
		self.rate = rospy.Rate(10) # 10Hz
		self.send_gear_cmd(Gear.D.value)

		while not rospy.is_shutdown():
			if(self.is_collision):
				self.send_gear_cmd(Gear.R.value)
				for _ in range(20):
					self.send_ctrl_cmd(0.4,0,10)
					self.rate.sleep()
				self.send_gear_cmd(Gear.D.value)
			else:
				self.send_ctrl_cmd(0.05,0,0)
				self.rate.sleep()

	def collision_callback(self, data):
		if len(data.collision_object) > 0:
			self.is_collision = True
			rospy.loginfo("Collision: %s", self.is_collision)
		else:
			self.is_collision = False

	def ego_callback(self, data):
		rospy.loginfo("Velocity: %.1f km/h", self.ego_status.velocity.x * 3.6)
		self.ego_status = data

	#기어 변경 이벤트 메세지 세팅 함수
	def send_gear_cmd(self, gear_mode):
		#기어 변경이 제대로 되기 위해서는 차량 속도가 약 0이어야 함.
		while abs(self.ego_status.velocity.x) > 0.1:
			self.send_ctrl_cmd(0,1,0)
			self.rate.sleep()
		gear_cmd = EventInfo()
		gear_cmd.option = 3
		gear_cmd.ctrl_mode = 3
		gear_cmd.gear = gear_mode
		self.event_cmd_srv(gear_cmd)

	#ctrl_cmd 메세지 세팅 함수
	def send_ctrl_cmd(self, accel, brake, steering):
		cmd = CtrlCmd()
		cmd.longlCmdType = 1
		cmd.accel = accel
		cmd.brake = brake
		cmd.steering = steering
		self.cmd_pub.publish(cmd)


class Gear(Enum):
	P, R, N, D = 1, 2, 3, 4


if __name__ == '__main__':
	try:
		CollisionAvoid()
	except rospy.ROSInterruptException:
	    pass
