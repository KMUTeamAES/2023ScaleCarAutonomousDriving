#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64,Int64

class BasicDriveNode:
    def __init__(self):
        rospy.init_node('basic_drive_node', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)#/commands/servo/position 
        self.pub_cmd_ang = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.rate = rospy.Rate(10)  # 주행 속도 조절을 위한 주기 (10Hz)
        self.speed_msg = Float64()
        self.angle_msg = Float64()

        ##variables
        ##variables_speed
    
        self.drive_speed = 1800
        self.static_speed = 2000
        self.dynamic_speed = 0
        self.child_speed = 1000
        self.rabacon_speed = 1000
        
        ##variables_ang
        self.drive_ang = 0.5
        self.static_right_ang = 0.8
        self.static_left_ang = 0.2
        self.dynamic_ang = 0.5
        self.lavacon_ang = 0.5
        self.rabacon_ang = 0.5
        self.stop_cunt = 0
        self.steer_ang = 0.5
        self.child_mark = 0
        self.obtacle_flag = 0
        
        self.turn_right_flag = 0
        self.turn_left_flag = 0

        self.mark_cunt = 0
        self.dynamic_cunt = 0
 
    def drive_with_steering(self, speed, steering_angle):
        self.speed_msg.data = speed
        self.angle_msg.data = steering_angle
        self.pub_cmd_vel.publish(self.speed_msg.data)
        self.pub_cmd_ang.publish(self.angle_msg.data)

    def run(self):
        rospy.Subscriber("steering_angle", Float64, self.drive_callback)
        rospy.Subscriber("status",Int64,self.status_CB)
        rospy.Subscriber("child",Int64,self.child_CB)
        # rospy.Subscriber("L_or_R",Int64,self.L_or_R_CB)
        while not rospy.is_shutdown():
            if self.obtacle_flag == 1:
                if self.turn_right_flag == 0:
                    self.turn_right_t1 = rospy.get_time()
                    self.turn_right_flag = 1
                t2 = rospy.get_time()
                while t2-self.turn_right_t1 <= 1.0: # 시간 확인 및 조절 필요
                    self.drive_with_steering(self.static_speed,self.static_right_ang)
                    t2 = rospy.get_time()
                while t2-self.turn_right_t1 <= 1.25 : # 시간 확인 및 조절 필요
                    self.drive_with_steering(self.static_speed,self.static_left_ang)
                    t2 = rospy.get_time()
                    rospy.sleep(5)
                self.turn_right_flag = 0

            elif self.obtacle_flag == 2 : #static_rignt
                print("MODE : Right Static")
                if self.turn_left_flag == 0:
                    self.turn_left_t1 = rospy.get_time()
                    self.turn_left_flag = 1
                t2 = rospy.get_time()

                while t2-self.turn_left_t1 <= 1.0: # 시간 확인 및 조절 필요
                    self.drive_with_steering(self.static_speed,self.static_left_ang)
                    t2 = rospy.get_time()
                while t2- self.turn_left_t1 <= 1.25 : # 시간 확인 및 조절 필요
                    self.drive_with_steering(self.static_speed,self.static_right_ang)
                    t2 = rospy.get_time()
                    rospy.sleep(5)
                self.turn_left_flag = 0
            else:##동적일 경우
                if self.dynamic_cunt <7:
                    print("dynamic")
                    rospy.sleep(2)
                    self.dynamic_cunt+=1
                else:
                    self.drive_with_steering(1500,self.steer_ang)

            self.rate.sleep()

    # def L_or_R_CB(self,msg):
        #1 -> 정적 왼쪽 -> 오른쪽으로 차선변경
        #2 -> 정적 오른쪽 -> 왼쪽으로 차선변경
        #3 -> 동적
        # self.obtacle_flag = msg.data
    
    def drive_callback(self, msg):
        self.steer_ang = msg.data
    
    def status_CB(self,msg):
        print("mark = ",self.child_mark)
        if msg.data == 0:#기본주행
            if self.child_mark == 101 or self.child_mark ==0:
                self.drive_with_steering(1500,self.steer_ang)
            elif self.child_mark == 100:
                self.drive_with_steering(1000,0.5)
        elif msg.data == 1 or msg.data == 2 or msg.data == 3:
            self.obtacle_flag = msg.data
        elif msg.data == 5:
            if self.child_mark == 100 and self.mark_cunt==0:
                self.drive_with_steering(0,0.5)
                rospy.sleep(6)
                self.mark_cunt+=1
            elif self.child_mark == 100 and self.mark_cunt!=0:
                self.drive_with_steering(1000,0.5)
            else:
                # self.drive_with_steering(1000,0.5)
                pass
        else:
            pass
    
    def child_CB(self,msg):
        # print("mark = ",msg.data)
        if msg.data == 100:
            self.child_mark = msg.data
        elif msg.data == 101:
            self.child_mark = msg.data
        else:
            pass


if __name__ == '__main__':
    try:
        node = BasicDriveNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
