#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class BasicDriveNode:
    def __init__(self):
        rospy.init_node('basic_drive_node', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)#/commands/servo/position 
        self.pub_cmd_ang = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.rate = rospy.Rate(20)  # 주행 속도 조절을 위한 주기 (10Hz)
        self.speed_msg = Float64()
        self.angle_msg = Float64()
 
    def drive_with_steering(self, speed, steering_angle):
        self.speed_msg.data = speed
        self.angle_msg.data = steering_angle
        self.pub_cmd_vel.publish(self.speed_msg.data)
        self.pub_cmd_ang.publish(self.angle_msg.data)

    def run(self):
        rospy.Subscriber("steering_angle", Float64, self.drive_callback)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def drive_callback(self, msg):
        steering_angle = msg.data

        # if steering_angle=>0 and steering_angle<1:
        self.drive_with_steering(1000,steering_angle)
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
