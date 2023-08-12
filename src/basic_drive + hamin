#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64,Int32

class BasicDriveNode:
    def __init__(self):
        rospy.init_node('basic_drive_node', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)#/commands/servo/position 
        self.pub_cmd_ang = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.rate = rospy.Rate(20)  # 주행 속도 조절을 위한 주기 (10Hz)
        self.speed_msg = Float64()
        self.angle_msg = Float64()
        self.types = Int32()
        # for obstacle
       
        self.turn_left_flag = 0
        self.turn_right_flag = 0
        self.dynamic_flag = 0
    def change_line_left(self) :
        self.drive_with_steering(2000,-0.2)

    def change_line_right(self) :
        self.drive_with_steering(2000,0.8)

    def stop(self) : 
        self.drive_with_steering(0,0)

    def drive_with_steering(self, speed, steering_angle):
        self.speed_msg.data = speed
        self.angle_msg.data = steering_angle
        self.pub_cmd_vel.publish(self.speed_msg.data)
        self.pub_cmd_ang.publish(self.angle_msg.data)

    def run(self):
        rospy.Subscriber("steering_angle", Float64, self.drive_callback)
        rospy.Subscriber("obtacles_detect",Int32,self.obstacle_callback)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def obstacle_callback(self,types):
        if types.data == 1: #정적, 왼쪽 차선 
            if self.turn_right_flag == 0:
                self.turn_right_t1 = rospy.get_time()
                self.turn_right_flag = 1
            t2 = rospy.get_time()
            while t2-self.turn_right_t1 <= 1.0: # 시간 확인 및 조절 필요
                self.change_line_right()
                t2 = rospy.get_time()
            while t2-self.turn_right_t1 <= 1.25 : # 시간 확인 및 조절 필요
                self.change_line_left()
                t2 = rospy.get_time()
            self.turn_right_flag = 0     

        elif types.data == 2: # 정적, 오른쪽 차선 
            if self.turn_left_flag == 0:
                self.turn_left_t1 = rospy.get_time()
                self.turn_left_flag = 1
            t2 = rospy.get_time()

            while t2-self.turn_left_t1 <= 1.0: # 시간 확인 및 조절 필요
                self.change_line_left()
                t2 = rospy.get_time()
            while t2- self.turn_left_t1 <= 1.25 : # 시간 확인 및 조절 필요
                self.change_line_right()
                t2 = rospy.get_time()
            self.turn_left_flag = 0       
                         
        elif types.data == 3: #동적 장애물 
            self.stop()
        #else:
            # 이때 일반 주행 하게 하고 싶음 
    def drive_callback(self, msg):
        steering_angle = msg.data

        # if steering_angle=>0 and steering_angle<1:
        self.drive_with_steering(2000,steering_angle)
        print("in range steering")
        # else:
        #    print("out of range steering")
        #    if steering_angle<0:
            #    self.drive_with_steering(1000,0.3)#steering angle 음수
        #    else:
            #    self.drive_with_steering(1000,0.7)#steering angle 1보다 큰 양수


if __name__ == '__main__':
    try:
        node = BasicDriveNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
