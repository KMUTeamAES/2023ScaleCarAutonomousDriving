#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from fiducial_msgs.msg import Fiducial, FiducialArray
from geometry_msgs.msg import Twist

class Sign():
    def __init__(self):
        self.sign_id = 0
        self.is_child_zone = False
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Float32, queue_size=10) # 속도 정보를 발행하는 곳
        self.bridge = CvBridge() # openCV와 ros 메시지 사이의 형식을 변환해주는 역할을 하는 객체
        self.mode_pub = rospy.Publisher('chlidren_zone', Int32, queue_size=10)

        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback) 
        #"/fiducial_vertices" 토픽에서 FiducialArray 메시지를 받아 어린이구역 정보 받음
        self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
        # 받은 어린이구역 정보 발행 (= ID 정보)
        self.pub_cnt = 0 # 신호 발행할 때마다 카운트 하는 변수 == publish 횟수

        self.show_image = True  # 이미지 창을 표시할지 여부를 설정합니다

    def child_sign_callback(self, data): # /fiducial_vertices 토픽에서 수신한 FiducialArray 메시지를 처리하는 함수
        if len(data.fiducials) > 0: # 신호가 들어오면 수행
            self.sign_id = data.fiducials[0].fiducial_id # 첫번째 신호의 id를 저장하고 발행
            self.sign_id_pub.publish(self.sign_id)
            self.pub_cnt = 0

            # 마커 인식 후 어린이 보호구역인지 확인 (마커 ID 100 어린이 보호구역)
            if self.sign_id == 100:
                self.is_child_zone = True
            elif self.sign_id == 101: # 끝나는 지점 마커 확인(마커 ID 101 어린이 보호구역 해제)
                self.is_child_zone = False
        else:
            self.pub_cnt += 1
            if self.pub_cnt > 20: 
                # 20회 이상 publishing을 한 경우에 id 0으로 초기화 하고 어린이 보호구역 해제 (이게 약간 일정 시간동안만 구역 유지하는 느낌) => * 수정 아닐 수 있음
                self.sign_id_pub.publish(0)
                self.pub_cnt = 0
                self.is_child_zone = False

    def run(self):
        rospy.init_node("sign_id") # ros의 노드를 sign_id 이름으로 초기화
        while not rospy.is_shutdown(): # 계속 반복 ros 꺼질 때 까지
            if self.is_child_zone:
                self.reduce_speed()
            else :
                self.basic_speed()

            if self.show_image:
                self.show_camera_image()

            rospy.sleep(0.1) # cpu 절약하려고

    def reduce_speed(self): # 이 부분은 basic_drive 파일에다가 옮겨서 구현해야함 => 그냥 여기다가 구현
        # 어린이 보호구역인 경우 속도를 줄이는 로직을 여기에 구현
        # msg = Float32()
        # msg.data = 0.1
        # self.pub_cmd_vel.publish(msg.data)
        mode = 100 # 1이 어린이 보호 구역 모드
        self.mode_pub.publish(mode)
    def basic_speed(self): # 어린이 보호 구역 해제 될 때, 원래 스피드로
        # msg = Float32()
        # msg.data = 1
        # self.pub_cmd_vel.publish(msg.data)
        mode = 101 # 0이 일반주행 모드
        self.mode_pub.publish(mode)

    def show_camera_image(self):
        # 카메라 이미지를 OpenCV 이미지로 변환
        try:
            image = self.bridge.imgmsg_to_cv2(rospy.wait_for_message("/camera_topic", Image), desired_encoding="bgr8")
            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)
        except rospy.exceptions.ROSException:
            pass

if __name__ == '__main__':
    try:
        node = Sign()
        node.run()
    except rospy.ROSInterruptException:
        pass
