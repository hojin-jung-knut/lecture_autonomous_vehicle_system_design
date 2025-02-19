#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# rospy 라이브러리와 필요한 메시지 유형(geometry_msgs.msg, turtlesim.msg)을 가져옵니다.
import rospy, rospkg, math, time, numpy as np
import pandas as pd
# from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# from nav_msgs.msg import Odometry
    
# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class turtlesim_ctrl:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "init_node"로 지정됩니다.
        rospy.init_node("init_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/turtle1/cmd_vel" 토픽에 Twist 메시지를 발행하는 퍼블리셔를 만듭니다.
        # "queue_size"는 메시지 대기열 크기를 나타냅니다.
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # ROS 2단계(필수): 퍼블리셔 설정

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/turtle1/pose" 토픽에서 Pose 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)  # ROS 2단계(필수): 서브스크라이버 설정
        # self.sub = rospy.Subscriber("/odom", Odometry, self.callback)

        # 메시지 발행 주기를 설정합니다. 이 예제에서는 10Hz로 설정합니다.
        self.rate = rospy.Rate(25)  # ROS 2-1단계(옵션): 발행 주기 설정
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path(pkg_name)
        # Twist 메시지 타입의 메시지 객체를 생성하고 초기화합니다.
        self.msg = Twist()  # 메시지 타입 설정 및 초기화
        self.t0 = time.time(); self.t = np.array([])
        self.x = np.array([]); self.y = np.array([])

        rospy.on_shutdown(self.shutdown_hook)
    
    def callback(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        # self.msg.linear.x = 1; self.msg.angular.z = -0.5
        # self.msg.linear.x = 0.5; self.msg.angular.z = -1
        # self.msg.linear.y = 1; self.msg.angular.z = -0.5
        self.msg.linear.x = math.sqrt(2)/3; self.msg.linear.y = math.sqrt(2)/3; self.msg.angular.z = -0.6
        # 퍼블리셔를 사용하여 Twist 메시지를 발행합니다.
        
        self.pub.publish(self.msg)  # ROS 3단계(필수): 퍼블리셔 - 퍼블리시 실행

        # print(f"msg: {msg}")
        # print(f"msg.x: {msg.x}, msg.y: {msg.y}")
        self.t = np.append(self.t, time.time()-self.t0)
        self.x = np.append(self.x, msg.x)
        self.y = np.append(self.y, msg.y)
        # print(msg.pose)
        # 지정한 발행 주기에 따라 슬립합니다. 이것은 메시지를 일정한 주기로 발행하기 위해 사용됩니다.
        self.rate.sleep()  # ROS 3-1단계(옵션): 퍼블리셔 - 주기 실행

    def shutdown_hook(self):  # Function to be called on shutdown
        self.x = np.around(self.x,2); self.y = np.around(self.y,2); self.t = np.around(self.t,4)
        print("Shutting down..."); print(f"x: {self.x,2}, y: {self.y,2}");
        print(self.x.max()); print(self.x.min())
        # print(self.x.mean())
        # data = np.array([self.t,self.x,self.y]); data = data.transpose()
        data = pd.DataFrame([self.t,self.x,self.y]); data = data.T
        # print(data)
        # np.savetxt("./turtle_pose.csv", data, delimiter=",")
        data.to_csv("./morai_ws/src/MORAI-DriveExample_ROS/turtle_path.csv")

# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    tc = turtlesim_ctrl()  # Class_Name 클래스의 인스턴스를 생성합니다.
    rospy.spin()

# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    try:
        main()
    except rospy.ROSInterruptException:
        pass