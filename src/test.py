import cv2
import numpy as np
import rospy
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

class LaneDetectorWebot:
    def __init__(self):
        rospy.init_node('lane_detect_webot')
        self.steer_angle_pub = rospy.Publisher('steering_angle', Float64, queue_size=10)
        rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_CB)  ##/camera1/usb_cam1/image_raw  /usb_cam/image_rect_color
        self.rate = rospy.Rate(20)
        #self.video = cv2.VideoCapture('lane_video.mp4')  # Webcam connection
        # Define source and destination points for perspective transform
        self.left_top = (285,270)
        self.left_bottom = (0,480)
        self.right_bottom = (640,480)
        self.right_top = (378,270)

        # self.left_top = (260,270)
        # self.left_bottom = (0,480)
        # self.right_bottom = (640,480)
        # self.right_top = (393,270)
        
        self.src_points = np.float32([self.left_top,self.left_bottom,self.right_bottom,self.right_top])
        # self.dst_points = np.float32([(320, 0), (320, 720), (960, 720), (960, 0)])
        self.dst_points = np.float32([(160, 0),(160, 480),(480, 480),(480, 0)])
        # Define other constants
        self.win_bottom_y = 0  # fixed value
        self.win_top_y = 432  # fixed value
        self.max_Theta = 0.1
        self.min_Theta = -0.1
        self.aver_Theta = 0.0
        self.default_angle = 0.5  # radian
        self.k1 = 0.01
        self.k2 = 0.01
        self.bridge = CvBridge()
        self.steer_msg = Float64()

    def image_CB(self,data):
        try:
            start = time.time()
            frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            height, width = frame.shape[:2]
            
            bird_eye_view = self.bird_eye_view_transform(frame, height, width)

            # bird_line_view = self.detect_lane_lines(bird_eye_view)
            
            white_image = self.image_filtering(bird_eye_view)

            
            left_x_base, right_x_base = self.histogram(white_image)
            
            out_image, left_bottom_x, left_top_x, right_bottom_x, right_top_x = self.sliding_windows(white_image,left_x_base, right_x_base)
            
            Theta = self.cal_ang(white_image,left_bottom_x,left_top_x,right_bottom_x,right_top_x)
            
            steer_angle = self.decion_steer_ang(Theta)
            print("steer_angle = ", steer_angle)
            
            
            # ref_distance = self.find_ref_distance(left_x_base, right_x_base)
            # ref_angle = self.find_ref_angle(left_bottom_x, left_top_x, right_bottom_x, right_top_x)
            # steer_angle = (self.k1 * ref_distance) + (self.k2 * ref_angle)
            # print("steer_angle = ", steer_angle)
            cv2.imshow("OG_frame", frame)

            # cv2.imshow("bird_eye_view", bird_eye_view)
            cv2.imshow("Result", out_image)

            # self.steer_msg.data = steer_angle
            self.steer_msg.data = steer_angle
            rospy.sleep(2)
            self.steer_angle_pub.publish(self.steer_msg.data)
            self.rate.sleep()
            # cv2.circle(frame, self.left_top, 5, (0,0,255), -1)
            # cv2.circle(frame, self.left_bottom, 5, (0,0,255), -1)
            # cv2.circle(frame, self.right_bottom, 5, (0,0,255), -1)
            # cv2.circle(frame, self.right_top, 5, (0,0,255), -1)
            # cv2.imshow("line_detect",bird_line_view)
                        
            end = time.time()
            print("end-start",end-start)

            cv2.waitKey(1)
            if 0xFF == ord('q'):
                cv2.destroyAllWindows()

        except Exception as e:
            print("Error:", e)

    def decion_steer_ang(self,Theta):
        if self.aver_Theta<Theta:#Theta가 0이상의 값 (오른쪽)
            if Theta>self.max_Theta:
                steer_angle = 0.9
            elif Theta>(self.max_Theta/2):
                steer_angle =0.7
            else:
                steer_angle = 0.5
        else:#Theta가 0이하의 값(왼쪽)
            if Theta<self.min_Theta:
                steer_angle = 0.1
            elif Theta<(self.min_Theta/2):
                steer_angle =0.3
            else:
                steer_angle = 0.5
        
        return steer_angle


    def detect_lane_lines(self,image): # 차선 검출 함수
    # 이미지 크기 추출
        height, width = image.shape[:2]

        # 이미지를 흑백으로 변환
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        edges = cv2.Canny(blur, threshold1=50, threshold2=150)

        # ROI (Region of Interest) 설정: 아래쪽만 남기고 나머지 부분은 제거
        roi_vertices = [(0, height), (width // 2, height // 2), (width, height)]
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, np.array([roi_vertices], dtype=np.int32), 255)
        # cv2.imshow("mask",mask)
        # cv2.imshow("edges",edges)
        masked_image = cv2.bitwise_and(edges, mask)#mask는 관심구역 edges는 edge검출한 list
        # 허프 변환 (Hough Transform)을 사용하여 직선 검출
        lines = cv2.HoughLinesP(masked_image, rho=1, theta=np.pi/180, threshold=5, minLineLength=100, maxLineGap=50)# 
        # print(lines)

        # 차선을 그릴 빈 이미지 생성
        line_image = np.zeros_like(image)

        # 검출된 직선을 이미지에 그리기
        # for line in lines:
            # x1, y1, x2, y2 = line[0]
            # cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=5)
        # if lines is None:
            # return []
        
        # 검출된 직선들을 하나의 다각형으로 그려 이진화된 이미지 생성
        if lines is not None:
            line_polygons = [lines.squeeze()]
            cv2.polylines(line_image, line_polygons, isClosed=False, color=(255, 255, 255), thickness=5)

        # cv2.imshow("line",line_image)
        # 원본 이미지와 차선 이미지를 합쳐서 반환
        # result_image = cv2.addWeighted(image, 0.8, line_image, 1, 0)

        return line_image
    
    def cal_ang(self,white_image,left_bottom_x,left_top_x,right_bottom_x,right_top_x):
        image_width = white_image.shape[1]
        
        left_half = white_image[:,:image_width//2]
        right_half = white_image[:,image_width//2:]

        #픽셀 수 합 계산
        left_pixel_sum = left_half.sum()
        right_pixel_sum = right_half.sum()
        print("left_pixel",left_pixel_sum)
        print("right_pixel",right_pixel_sum)
        print("sum_pixel",left_pixel_sum+right_pixel_sum)
        
        # if left_pixel_sum>9000
        Theta_right = (right_top_x - right_bottom_x) / (self.win_top_y - self.win_bottom_y)
        # Theta_left = (left_top_x - left_bottom_x) / (self.win_top_y - self.win_bottom_y)
        # print("ang",Theta_left)

        if right_pixel_sum<90000:
            Theta_left = (left_top_x - left_bottom_x) / (self.win_top_y - self.win_bottom_y)
            print("left")
            return Theta_left
        print("right")
        return Theta_right
    
    def find_ref_distance(self, left_x_base, right_x_base):
        ref_D_left = (320 - left_x_base) * -1
        ref_D_right = right_x_base - 320
        print("ref_D_left",ref_D_left)
        print("ref_D_right",ref_D_right)
        return ref_D_left + ref_D_right

    def find_ref_angle(self, left_bottom_x, left_top_x, right_bottom_x, right_top_x):
        win_bottom_y = 0  # fixed value
        win_top_y = 432  # fixed value
        Theta_left = (left_top_x - left_bottom_x) / (win_top_y - win_bottom_y)
        Theta_right = (right_top_x - right_bottom_x) / (win_top_y - win_bottom_y)
        print("left_ang",Theta_left)
        return Theta_left - Theta_right

    def sliding_windows(self, binary_img,left_x_base,right_x_base):
        # The sliding windows logic goes here
        if binary_img is None:
            pass
        n_win = 5  # 좌,우 차선별 탐지 윈도우의 개수, 적어지면 샘플링이 적어지는 샘이라서 급커브 같은데서 영역을 정확히 못잡아냄
        window_height = np.int(binary_img.shape[0] / n_win)  # 윈도우 높이
        non_zero = binary_img.nonzero()  # binary_img에서 값이 0 이 아닌 픽셀들의 좌표를 x 좌표 y 좌표로 각각 인덱싱해서 배출. 예를들어 0,0의 픽셀값이 0이 아니라면 array([array[0], array[0]]) 형태
        non_zero_y = np.array(non_zero[0])  # 0이아닌 y좌표 1차원 튜플형태
        non_zero_x = np.array(non_zero[1])  # 0이아닌 x좌표
        out_img = np.dstack((binary_img, binary_img, binary_img)) * 255

        left_x_current = left_x_base
        right_x_current = right_x_base

        margin = 50  # 윈도우 margin
        min_pix = 50

        left_lane_indices = []
        right_lane_indices = []

        for window in range(n_win):
            win_y_low = binary_img.shape[0] - (window + 1) * window_height  # 고정된 값
            win_y_high = binary_img.shape[0] - window * window_height  # 고정된 값

            # 좌측차선의 윈도우 위 아래 x좌표
            win_x_left_low = left_x_current - margin  # 히스토그램에서 받아온 x좌표에서 margin만큼 더하고 뺌
            win_x_left_high = left_x_current + margin  # 히스토그램에서 받아온 x좌표에서 margin만큼 더하고 뺌

            # 우측 차선의 윈도우 위 아래 x 좌표
            win_x_right_low = right_x_current - margin  # 마찬가지
            win_x_right_high = right_x_current + margin  # 마찬가지

            cv2.rectangle(out_img, (win_x_left_low, win_y_low), (win_x_left_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_x_right_low, win_y_low), (win_x_right_high, win_y_high), (0, 255, 0), 2)

            good_left_indices = ((non_zero_y >= win_y_low) & (non_zero_y < win_y_high) &
                                 (non_zero_x >= win_x_left_low) & (non_zero_x < win_x_left_high)).nonzero()[0]
            good_right_indices = ((non_zero_y >= win_y_low) & (non_zero_y < win_y_high) &
                                  (non_zero_x >= win_x_right_low) & (non_zero_x < win_x_right_high)).nonzero()[0]

            # 위에서 추려낸 값을 append
            left_lane_indices.append(good_left_indices)
            right_lane_indices.append(good_right_indices)

            # 다음 윈도우 위치 업데이트
            if len(good_left_indices) > min_pix:
                left_x_current = np.int(np.mean(non_zero_x[good_left_indices]))

            if len(good_right_indices) > min_pix:
                right_x_current = np.int(np.mean(non_zero_x[good_right_indices]))
                # print("window = ",window)
            if window==0:
                left_bottom_x = (win_x_left_low+win_x_left_high)/2 #left_x_current
                right_bottom_x = (win_x_right_low+win_x_right_high)/2 #right_x_current
                # print("0  left_bottom_x = ",left_bottom_x)
                # print("0  right_bottom_x = ",right_bottom_x)

            elif window==4:
                left_top_x = (win_x_left_low+win_x_left_high)/2 #left_x_current
                right_top_x = (win_x_right_low+win_x_right_high)/2 #right_x_current
                # print("9  left_bottom_x = ",left_bottom_x)
                # print("9  right_bottom_x = ",right_bottom_x)


        # 배열 합치기
        # 이 부분은 디텍팅 된 차선의 픽셀의 좌표 집합임.
        left_lane_indices = np.concatenate(left_lane_indices)
        right_lane_indices = np.concatenate(right_lane_indices)

        # 좌 우측 라인의 픽셀 위치들을 추출
        left_x = non_zero_x[left_lane_indices]  # 1차원 배열 형태로 저장
        left_y = non_zero_y[left_lane_indices]  # 1차원 배열 형태로 저장
        right_x = non_zero_x[right_lane_indices]  # 1차원 배열 형태로 저장
        right_y = non_zero_y[right_lane_indices]  # 1차원 배열 형태로 저장

        # 다항식으로 피팅한 좌표들을 2차다항식으로 피팅
        try:
            left_fit = np.polyfit(left_y, left_x, 2)  # 원래는 검출된 차선을 원본 이미지에 그리기 위해 피팅을 사용한 것으로 보임.
            right_fit = np.polyfit(right_y, right_x, 2)
        except np.linalg.LinAlgError as e:
            print("Exception occurred:", e)
            return out_img, 0, 0, 0, 0
        
        ploty = np.linspace(0, binary_img.shape[0] - 1, binary_img.shape[0])
        # left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        # right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        out_img[non_zero_y[left_lane_indices], non_zero_x[left_lane_indices]] = [255, 255, 255]
        out_img[non_zero_y[right_lane_indices], non_zero_x[right_lane_indices]] = [255, 255, 255]


        return out_img, left_bottom_x, left_top_x, right_bottom_x, right_top_x

    def histogram(self, frame):
        # The histogram logic goes here
        histogram_left = np.sum(frame[frame.shape[0]//2:,:frame.shape[1]//2], axis=0)
        histogram_right = np.sum(frame[frame.shape[0]//2:,frame.shape[1]//2:], axis=0)

        left_x_base = np.argmax(histogram_left)
        right_x_base = np.argmax(histogram_right)+frame.shape[1]//2

        return left_x_base, right_x_base

    def image_filtering(self, warped_frame):
        # The image filtering logic goes here
        hsv = cv2.cvtColor(warped_frame,cv2.COLOR_BGR2HSV)
        # mask_white = cv2.inRange(hsv,(0,0,150),(180,30,255))
        mask_white = cv2.inRange(hsv,(0,0,150),(180,30,255))
        # cv2.imshow("sum_mask",mask_white)
        return mask_white

    def bird_eye_view_transform(self, frame, height, width):
        M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        warp_frame = cv2.warpPerspective(frame,M,(width,height))
        return warp_frame

    # def process_frame(self): ##여기선 사용하지 않음 -> image callback함수에서 process_frame역할 대체
        # while self.video.isOpened():
            # ret, frame = self.video.read()
# 
            # if not ret:
                # break
# 
            # height, width = frame.shape[:2]
            # result_hough = self.detect_lane_lines(frame)
            # cv2.imshow("result_hough",result_hough)
            # bird_eye_view = self.bird_eye_view_transform(frame, height, width)
            # bird_line_view = self.image_filtering(bird_eye_view)
# 
            # cv2.imshow("Video", frame)
            # cv2.imshow("bird_eye_view", bird_eye_view)
# 
            # left_x_base, right_x_base = self.histogram(bird_line_view)
            # out_image, left_bottom_x, left_top_x, right_bottom_x, right_top_x = self.sliding_windows(bird_line_view,left_x_base, right_x_base)
            # ref_distance = self.find_ref_distance(left_x_base, right_x_base)
            # ref_angle = self.find_ref_angle(left_bottom_x, left_top_x, right_bottom_x, right_top_x)
            # steer_angle = (self.k1 * ref_distance) + (self.k2 * ref_angle)
            # print("steer_angle = ", steer_angle)
# 
            # if 0.0 < steer_angle < 1.0:
                # steer_angle_msg = Float64()
                # steer_angle_msg.data = steer_angle
                # self.steer_angle_pub.publish(steer_angle_msg)
                # self.rate.sleep()
# 
            # cv2.imshow("Result", out_image)
# 
            # if cv2.waitKey(30) & 0xFF == ord('q'):
                # break
# 
        # self.video.release()
        # cv2.destroyAllWindows()
def main():
    sub_image = LaneDetectorWebot()
    rospy.spin()

    # sub_image.image_CB()
    # sub_image.process_frame()

if __name__ == "__main__":
    main()
    # lane_detector = LaneDetectorWebot()