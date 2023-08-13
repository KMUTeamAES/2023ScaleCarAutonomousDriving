#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64,Int64,Int32

class BasicDriveNode:
    def __init__(self):
        rospy.init_node('basic_drive_node', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)#/commands/servo/position 
        self.pub_cmd_ang = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.rate = rospy.Rate(15)  # 주행 속도 조절을 위한 주기 (10Hz)
        self.speed_msg = Float64()
        self.angle_msg = Float64()
        rospy.Subscriber("steering_angle", Float64, self.Drive_CB)
        rospy.Subscriber("status",Int64,self.Obtacle_CB)
        rospy.Subscriber("rabacon_mode",Int32,self.rabacon_CB)
        rospy.Subscriber("children_zone", Int32, self.Child_CB)
        rospy.Subscriber("rabacon_steer", Float64, self.Child_steer_CB)
        #라바콘 steer 토픽 
        #라바콘 색상 판단 토픽
        
         
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
        self.child_ang = 0.5
        self.rabacon_ang = 0.5
        
        ##obtacles
        self.obtacle_flag = 0
        self.turn_right_flag = 0
        self.turn_left_flag = 0
        
        ##rabacon
        self.rabacon_flag = 0
        self.rabacon_color = 0

        ##children
        self.children_flag = 0
    def drive_with_steering(self, speed, steering_angle):
        self.speed_msg.data = speed
        self.angle_msg.data = steering_angle
        self.pub_cmd_vel.publish(self.speed_msg.data)
        self.pub_cmd_ang.publish(self.angle_msg.data)
        print(f"SPEED : {speed}")
        print(f"ANGLE : {steering_angle}")
    

    def Drive_CB(self, msg):
        self.drive_ang = msg.data

        # self.drive_with_steering(1000,steering_angle)
        # print("in range steering")
    
    def Obtacle_CB(self,msg):
        self.obtacle_flag = msg.data
        
    def rabacon_CB(self,msg):
        self.rabacon_flag = msg.data
    
    def Child_CB(self,msg):
        self.children_flag = msg.data
    
    def Child_steer_CB(self,msg):
        self.child_ang = msg.data
                
    def change_line_left(self) :
        self.drive_with_steering(self.static_speed,self.static_left_ang)

    def change_line_right(self) :
        self.drive_with_steering(self.static_speed,self.static_right_ang)

    def stop(self) : 
        self.drive_with_steering(self.dynamic_speed,self.dynamic_ang)
        
    def children_drive(self):
        self.drive_with_steering(self.child_speed,self.child_ang) 
    
    def rabacon_drive(self):
        self.drive_with_steering(self.rabacon_speed,self.rabacon_ang) 
        
    def run(self):
        
        while not rospy.is_shutdown():
            self.rate.sleep()
            
           
        if self.rabacon_color == 1:
            if self.rabacon_flag == 1:
                self.rabacon_drive()
                print("MODE : rabacon")
            else:
            #just drive
                self.drive_with_steering(self.drive_speed,self.drive_ang)     
                print("MODE : Just Drive")   
        else:
        
            if self.obtacle_flag == 1  : #static_left
                print("MODE : Left Static")
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
        
            elif self.obtacle_flag == 2 : #static_rignt
                print("MODE : Right Static")
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
                
            #dynamic_obtlaces    
            elif self.obtacle_flag == 3  : 
                self.stop()
                print("MODE : Dynamic") 
            
            # children_zone    
            elif self.children_flag == 1:
                self.children_drive()
                print("MODE : Children Zone")
            
            #just drive
            else:
                self.drive_with_steering(self.drive_speed,self.drive_ang)
                print("MODE : Just Drive")   
               
                        

if __name__ == '__main__':
    try:
        node = BasicDriveNode()
        rospy.Timer(rospy.Duration(1.0/30.0), node.run) 
        #node.run()
    except rospy.ROSInterruptException:
        pass
