import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64,Int64,Bool
from fiducial_msgs.msg import Fiducial, FiducialArray

class lane_detect:
	def __init__(self):
		rospy.init_node('lane_detection')
		self.steer_angle_pub = rospy.Publisher('steering_angle', Float64, queue_size=10)
		self.status_pub = rospy.Publisher('status', Int64, queue_size=10)
		# self.L_or_R_pub = rospy.Publisher('L_or_R', Int64, queue_size=10)
		self.child_pub = rospy.Publisher('child',Int64,queue_size=10)
		rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_CB)
		rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback)
		# rospy.Subscriber("objec_flag",Int64,self.object_CB)
		self.bridge = CvBridge()
		self.rate = rospy.Rate(100)

		##msgs
		self.steer_msg = Float64()
		self.status_msg = Int64()
		self.L_or_R_msg = Bool()
		self.child_msg = Int64()
		
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
		self.ob_flag = 0

	def image_CB(self,data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
			height, width = frame.shape[:2]
			bird_view = self.bird_eye_view(frame, height, width)
			self.find_lanes(bird_view)
			# if self.status_msg.data == 0:
				# if self.ob_flag ==1:
					# self.decision_L_or_R(bird_view)
					# self.ob_flag = 0
				# elif self.ob_flag ==2:
					# self.status_msg.data = 3
				# self.L_or_R_pub.publish(self.L_or_R_msg.data)
			# cv2.imshow("OG_frame",frame)
			# cv2.imshow('bird_view',bird_view)
			self.status_pub.publish(self.status_msg.data)
			key = cv2.waitKey(1)
			if key ==ord('q'):
				rospy.signal_shutdown("User requested shutdown")
		except Exception as e:
			print("Error:", e)

	def object_CB(self,msg):#1이 정적 2가 동적
		msg.data = self.ob_flag
		# if msg.data==1:
			# self.ob_flag = msg.data
		# elif msg.data==2:
			# self.ob_flag = msg.data
		# else:
			# pass
	def histogram(self,frame):
    
		histogram_left = np.sum(frame[frame.shape[0]//2:,:frame.shape[1]//2], axis=0)
		histogram_right = np.sum(frame[frame.shape[0]//2:,frame.shape[1]//2:], axis=0)

		left_x_base = np.argmax(histogram_left)
		right_x_base = np.argmax(histogram_right)+frame.shape[1]//2
    	
		return left_x_base,right_x_base
	def child_sign_callback(self,data):
		if len(data.fiducials) > 0: # 신호가 들어오면 수행
			self.child_msg.data = data.fiducials[0].fiducial_id # 첫번째 신호의 id를 저장하고 발행
			self.child_pub.publish(self.child_msg.data)
		else:
			pass

	def bird_eye_view(self,frame,height,width):
		M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
		warp_frame = cv2.warpPerspective(frame,M,(width,height))
		return warp_frame
	
	def decision_L_or_R(self,image):##왼쪽 차선 오른쪽 차선 판단
		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		mask_white = cv2.inRange(hsv,(0,0,200),(180,30,255))
		
		height, width = mask_white.shape[:2]

		left_half = mask_white[:,:width//2]
		right_half = mask_white[:,width//2:]

		left_pixel_count = cv2.countNonZero(left_half)
		right_pixel_count = cv2.countNonZero(right_half)
		print("left = ",left_pixel_count)
		print("right = ",right_pixel_count)

		if left_pixel_count > right_pixel_count:
			self.status_msg.data = 1
			print("왼쪽입니다")
		else:
			self.status_msg.data = 2
			print("오른쪽입니다")
		# self.L_or_R_pub.publish(self.L_or_R_msg.data)

	def laba_decision(self,frame):
		hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
		mask_red = cv2.inRange(hsv,(0,100,100),(10,255,255))
		red_pixel_count = cv2.countNonZero(mask_red)

	def find_lanes(self,image):
		# 그레이스케일 이미지로 변환
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	    # 엣지 검출
		edges = cv2.Canny(gray,threshold1=50, threshold2=150)
		
		# left_x_base,right_x_base = self.histogram(image)
		# print("left",left_x_base)
		# print("right",right_x_base)
	    # ROI(Region of Interest) 설정
		height, width = edges.shape[:2]
		# roi_vertices = [(0, height), (width / 2, 0), (width, height)]
		roi_vertices = [(0, 380),(0,260), (width, 260), (width, 380)]
		mask = np.zeros_like(edges)
		cv2.fillPoly(mask, np.array([roi_vertices], np.int32), 255)
		masked_edges = cv2.bitwise_and(edges, mask)

		# roi_left_half = masked_edges[:,0:width//2]
		# roi_right_half = masked_edges[:,width//2:]

	    # 허프 변환을 통한 직선 검출
		lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)
		#threshold값 낮을수록 직선 더 잘 검출 
		#lines라는 변수는 2차원 배열 (x1,y1,x2,y2)가 행에 저장돼 있음. 즉 무한 by 4 형태
		data = []
		# line_image = np.zeros_like(image)
		if lines is not None:
			print("line found")
			for line in lines:
				x1, y1, x2, y2 = line[0]
				# cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
		
				if x2 - x1 !=0:
					slope = (y2-y1) / (x2-x1)
					slope_degree = np.arctan(slope)*180/np.pi

					data.append(x1)
					# print("slope_degree",slope_degree)
					if abs(slope)>0.5:
						self.status_msg.data = 0
						# if slope_degree < 0: #우회전
							# if abs(slope_degree)>self.aver_ang-3:
								# self.steer_msg.data = self.aver_steer
							# elif abs(slope_degree) >80:
								# self.steer_msg.data = self.aver_steer+0.2
							# else:
								# self.steer_msg.data = self.aver_steer+0.4
						# else: #좌회전
							# if slope_degree>self.aver_ang-3:
								# self.steer_msg.data = self.aver_steer
							# elif slope_degree>80:
								# self.steer_msg.data = self.aver_steer-0.2
							# else:
								# self.steer_msg.data = self.aver_steer-0.4
					else:
						self.status_msg.data = 0

				# self.steer_msg.data = slope_degree*0.005 + 0.5 # 그냥 값 그대로 넣기
			# if (slope_degree*0.005 + 0.5) < 0:
			# 	self.steer_msg.data = 0
			# elif (slope_degree*0.005 + 0.5) >1 :
			# 	self.steer_msg.data = 1

				if abs(max(data)-min(data))<30:
					if min(data) > 320:
						self.steer_msg.data = self.aver_steer - 0.2
					else :
						self.steer_msg.data = self.aver_steer + 0.35
						# print("enter")
				else :
					if abs(max(data)-320) > abs(320-min(data)) :
						self.steer_msg.data = self.aver_steer + 0.2
						# print("check")
					elif abs(max(data)-320) < abs(320-min(data)) :
						self.steer_msg.data = self.aver_steer - 0.2
						# print("double check")
		else:
			print("line not found")
		print("steering_angle",self.steer_msg.data)
		self.steer_angle_pub.publish(self.steer_msg.data)
		# self.status_pub.publish(self.status_msg.data)
		self.rate.sleep()
		#원본 이미지와 직선 이미지 합치기
		# result = cv2.addWeighted(image,0.8,line_image,1,0)
		# cv2.imshow("Hough_lane",result)
		
def main():
	start = lane_detect()
	rospy.spin()
	cv2.destroyAllWindows() 

if __name__=="__main__":
	main()
