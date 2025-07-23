#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd

class S_Drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        # /ctrl_cmd 토픽 발행
        cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)
        rate = rospy.Rate(10) # 10Hz
        # CtrlCmd 메세지 생성
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.accel = 0.1
        steering_cmd = (-0.1,0.1) # wheel steer angle(rad)
        cmd_cnts = 10 # 토픽 전송 수
        
        while not rospy.is_shutdown():
            for i in range(2):
                # 0 : 우회전, 1 : 좌회전
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
                    rate.sleep()

if __name__ == '__main__':
    try:
        S_Drive()
    except rospy.ROSInterruptException:
        pass
