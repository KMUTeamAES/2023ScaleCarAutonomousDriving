#!/usr/bin/env python3

# 모듈 가져오기
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from math import *
import os

# 클래스 생성
class E_STOP: 

# 초기화 및 초기 설정 
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("object_detection")
        # 노드 역할 설정
        self.ctrl_pub = rospy.Publisher("objec_flag",Int64,queue_size=10)
        rospy.Subscriber("/scan",LaserScan,self.lidar_CB)
        rospy.Subscriber("L_or_R",Bool,self.LR_CB)
        lidar_msg = LaserScan()
        self.direct_msg = Int64()
        
        
    # 함수 설정
    def LR_CB(self,direct):
        self.direct_msg.data = direct.data
            
    def lidar_CB(self,msg): 
        static_max_range = 0.6 
        static_max_degree = 165 # 좌우 180 - static_max_degree 범위 ex : 150 -> -30 ~ 30 
        dynamic_max_range = 0.5 
        dynamic_max_degree = 130 # 정적은 좁게 멀리, 동적은 넓게 가까이 
        dynamic_check = 0
        static_check = 0
        #avoid_degree = []
        #avoid_index=[]
        ctrl_msg = Int64() # 0 : 정상 , 1 : 정적(왼쪽 차선) , 2 : 정적(오른쪽 차선), 3 : 동적 
        os.system('clear')
        degrees = [(msg.angle_min + msg.angle_increment*index)*180/pi for index, value in enumerate(msg.ranges)]
        # print(degrees)
        for index, value in enumerate(msg.ranges):
            if abs(degrees[index]) > static_max_degree  and  dynamic_max_range < msg.ranges[index] < static_max_range:
                static_check += 1
                #avoid_degree.append(degrees[index])
                #avoid_index.append(index)
            # elif abs(degrees[index]) > dynamic_max_degree  and  0 < msg.ranges[index] < dynamic_max_range:
                # dynamic_check += 1
            else:
                pass
            print(value)

       # if e_stop_check>10:
         #   ctrl_msg = 0
        # 인덱스의 수 -> 측정된 각도의 갯수 의미 -> 물체의 크기

        
        if static_check > 7.5: # 측정하면서 값 수정 , 물체크기 
            ctrl_msg = 1
            # if self.direct_msg.data == True: # left
                # print("Left_Static_Obtacle")
                # ctrl_msg = 1
            # else:
                # print("Rigth_Static_Obtacle")
                # ctrl_msg = 2
            
        # elif dynamic_check > 10: # 측정하면서 값 수정 , 물체크기 
            # print("Dynamic_Obtacle")
            # ctrl_msg = 1
        else :
            print("Safe")
            ctrl_msg = 0
        print(ctrl_msg)
        #print(msg.ranges)
        self.ctrl_pub.publish(ctrl_msg)

def main():
    e_stop=E_STOP()
    rospy.spin()

if __name__=="__main__":
    main()

