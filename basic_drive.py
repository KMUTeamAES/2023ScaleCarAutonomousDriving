#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class BasicDriveNode:
    def __init__(self):
        rospy.init_node('basic_drive_node', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)#/commands/servo/position 
        self.pub_cmd_ang = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.rate = rospy.Rate(10)  # 주행 속도 조절을 위한 주기 (10Hz)

    def drive_straight(self, speed):
        twist_msg = Float64()
        twist_msg.data = speed
        ang_msg = Float64()
        ang_msg.data = 0.5
        # twist_msg.data = 0.0
        self.pub_cmd_vel.publish(twist_msg)
        self.pub_cmd_ang.publish(ang_msg)

    def drive_with_steering(self, speed, steering_angle):
        twist_msg = Float64()
        twist_msg.data = speed
        ang_msg = Float64()
        ang_msg.data = steering_angle
        # twist_msg.data = steering_angle
        self.pub_cmd_vel.publish(twist_msg)
        self.pub_cmd_ang.publish(ang_msg)

    def run(self):
        rospy.Subscriber("steering_angle", Float64, self.drive_callback)

        while not rospy.is_shutdown():
            self.rate.sleep()
            ##임시
            #self.drive_straight(1000)
            #print("straight_drive_mode")

    def drive_callback(self, msg):
        steering_angle = msg.data

        if abs(steering_angle) < 0.01:  # 아주 작은 값이면 기본 직진 주행
           self.drive_straight(1000)  # 여기서 0.2는 기본 주행 속도 (수정 가능)
           print("straight")
        else:
           self.drive_with_steering(1000, steering_angle)  # 여기서 0.2는 주행 속도 (수정 가능)
           print("not straight")


if __name__ == '__main__':
    try:
        node = BasicDriveNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
