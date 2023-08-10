import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class lane_detect:
	def __init__(self):
		rospy.init_node('hojun')
		self.steer_angle_pub = rospy.Publisher('steering_angle', Float64, queue_size=10)
		rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_CB)
		self.bridge = CvBridge()
		self.rate = rospy.Rate(10)

		##msgs
		self.steer_msg = Float64()
		
		##bird_variable
		self.left_top = (285,270)
		self.left_bottom = (0,480)
		self.right_bottom = (640,480)
		self.right_top = (378,270)
		self.src_points = np.float32([self.left_top,self.left_bottom,self.right_bottom,self.right_top])
		self.dst_points = np.float32([(160, 0),(160, 480),(480, 480),(480, 0)])
		
		##default value
		self.aver_ang = 90
		self.aver_steer = 0.5

	def image_CB(self,data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
			height, width = frame.shape[:2]
			bird_view = self.bird_eye_view(frame, height, width)
			lines = self.find_lanes(bird_view)
					
			cv2.imshow("OG_frame",frame)
			cv2.imshow('bird_view',bird_view)
			key = cv2.waitKey(1)
			if key ==ord('q'):
				rospy.signal_shutdown("User requested shutdown")
		except Exception as e:
			print("Error:", e)
	# def calculate_angle(self,lines):
		# for line in lines:


	# def calculate_steering_angle()
	def bird_eye_view(self,frame,height,width):
		M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
		warp_frame = cv2.warpPerspective(frame,M,(width,height))
		return warp_frame

	def find_lanes(self,image):
	    # 그레이스케일 이미지로 변환
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	
	    # 엣지 검출
		edges = cv2.Canny(gray,threshold1=50, threshold2=150)

	    # ROI(Region of Interest) 설정
		height, width = edges.shape[:2]
		# roi_vertices = [(0, height), (width / 2, 0), (width, height)]
		roi_vertices = [(0, 380),(0,0), (width, 0), (width, 380)]
		mask = np.zeros_like(edges)
		cv2.fillPoly(mask, np.array([roi_vertices], np.int32), 255)
		masked_edges = cv2.bitwise_and(edges, mask)
	
	    # 허프 변환을 통한 직선 검출
		lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)
		#threshold값 낮을수록 직선 더 잘 검출 
		#lines라는 변수는 2차원 배열 (x1,y1,x2,y2)가 행에 저장돼 있음. 즉 무한 by 4 형태
		line_image = np.zeros_like(image)
		if lines is not None:
			print("line found")
			for line in lines:
				x1, y1, x2, y2 = line[0]
				cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
				# print("x1",x1)
				# print("x2",x2)
				if x2 - x1 !=0:
					slope = (y2-y1) / (x2-x1)
					slope_degree = np.arctan(slope)*180/np.pi
					print("slope_degree",slope_degree)
					if slope_degree < 0: #우회전
						if abs(slope_degree)>self.aver_ang-5:
							self.steer_msg.data = self.aver_steer
						else:
							self.steer_msg.data = self.aver_steer+0.2
					else: #좌회전
						if slope_degree>self.aver_ang-5:
							self.steer_msg.data = self.aver_steer
						else:
							self.steer_msg.data = self.aver_steer-0.2
		else:
			print("line not found")
		self.steer_angle_pub.publish(self.steer_msg.data)
		self.rate.sleep()
		#원본 이미지와 직선 이미지 합치기
		result = cv2.addWeighted(image,0.8,line_image,1,0)
		# cv2.circle(result, (320,380), 5, (0,255,0), 2)
		cv2.imshow("Hough_lane",result)
		# cv2.waitKey(1)
		# cv2.destroyAllwindows()
		return lines

def main():
	start = lane_detect()
	rospy.spin()
	cv2.destroyAllWindows() 

if __name__=="__main__":
	main()