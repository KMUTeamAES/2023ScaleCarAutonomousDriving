import cv2
import numpy as np
import rospy 
from std_msgs.msg import Float64

rospy.init_node('lane_detection')
steer_angle_pub = rospy.Publisher('steering_angle', Float64, queue_size=10)
rate = rospy.Rate(10)

video = cv2.VideoCapture('lane_video.mp4')

##functions
## draw_sliding_windows 이건 안 봐도 됨
# def draw_sliding_windows(frame, left_fit, right_fit, margin, height, n_win):
#    빈 캔버스 생성 (원본 영상 위에 그림을 그리기 위해)
    # window_canvas = np.zeros_like(frame)
    # window_height = np.int(height / n_win)  # 윈도우 높이
# 
 #   좌측차선의 윈도우와 우측 차선의 윈도우에 대해
    # for window in range(n_win):
        # win_y_low = height - (window + 1) * window_height
        # win_y_high = height - window * window_height
# 
  #      좌측차선의 윈도우 위 아래 x좌표
        # win_x_left_low = left_fit[0] * win_y_low ** 2 + left_fit[1] * win_y_low + left_fit[2] - margin
        # win_x_left_high = left_fit[0] * win_y_high ** 2 + left_fit[1] * win_y_high + left_fit[2] + margin
# 
   #     우측차선의 윈도우 위 아래 x좌표
        # win_x_right_low = right_fit[0] * win_y_low ** 2 + right_fit[1] * win_y_low + right_fit[2] - margin
        # win_x_right_high = right_fit[0] * win_y_high ** 2 + right_fit[1] * win_y_high + right_fit[2] + margin
# 
    #    윈도우 영역을 다각형 형태로 정의
        # left_pts = np.array([[win_x_left_low, win_y_low], [win_x_left_high, win_y_high]], np.int32)
        # right_pts = np.array([[win_x_right_low, win_y_low], [win_x_right_high, win_y_high]], np.int32)
        # pts = np.concatenate((left_pts, right_pts[::-1]))
# 
     #   윈도우 영역을 캔버스 위에 그립니다.
        # cv2.fillPoly(window_canvas, [pts], (0, 255, 0))
# 
 #   원본 영상 위에 윈도우 캔버스를 합성합니다.
    # result = cv2.addWeighted(frame, 1, window_canvas, 0.3, 0)
# 
    # return result
def find_ref_distance(left_x_base,right_x_base):
    ref_D_left=(640-left_x_base)*-1
    ref_D_right=right_x_base-640
    # print("ref_D_left",ref_D_left)
    # print("ref_D_right",ref_D_right)
    return ref_D_left + ref_D_right
def find_ref_angle(left_bottom_x, left_top_x, right_bottom_x, right_top_x):
    win_bottom_y = 0#고정값
    win_top_y = 648#고정값
    Theta_left = (left_top_x - left_bottom_x)/(win_top_y - win_bottom_y)
    Theta_right = (right_top_x - right_bottom_x)/(win_top_y - win_bottom_y)

    return Theta_left - Theta_right
    
def SlidingWindows(binary_img):
    n_win = 10  # 좌,우 차선별 탐지 윈도우의 개수, 적어지면 샘플링이 적어지는 샘이라서 급커브 같은데서 영역을 정확히 못잡아냄
    window_height = np.int(binary_img.shape[0] / n_win)  # 윈도우 높이
    non_zero = binary_img.nonzero()  # binary_img에서 값이 0 이 아닌 픽셀들의 좌표를 x 좌표 y 좌표로 각각 인덱싱해서 배출. 예를들어 0,0의 픽셀값이 0이 아니라면 array([array[0], array[0]]) 형태
    non_zero_y = np.array(non_zero[0])  # 0이아닌 y좌표 1차원 튜플형태
    non_zero_x = np.array(non_zero[1])  # 0이아닌 x좌표
    out_img = np.dstack((binary_img, binary_img, binary_img)) * 255

    left_x_current = left_x_base
    right_x_current = right_x_base

    margin = 100  # 윈도우 margin
    min_pix = 50

    left_lane_indices = []
    right_lane_indices = []

    for window in range(n_win):
        # 각 윈도우는 버드아이뷰 상단점을 기준으로 y 윈도우들의 좌표값을 구한다.
        # win_y_low는 이미지 최상단 y좌표 (height)에서 window+1 에 heght를 곱하면 그만큼 아래쪽이며
        # win_y_high 는 그 low 좌표와 짝을 이루는 위쪽 좌표이므로 window 에 height를 곱한다.
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
        """
        다음 아래 두 식은 다음과 같은 연산을 진행함.
        non_zero_y 의 모든 좌표 중 현재 window의 y 최소값, 최대값 보다 큰값에 대한 판별을 진행한 TF 테이블을 만들고
        x에 대해서도 같은 방식을 진행하여 TF 테이블을 만든다. 이 값들이 모두 T인 지점들은 1이 나오므로
        해당 점들을 non_zero 로 뽑아내고 x축 값만을 취함
        """

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
        if window==0:
            left_bottom_x = (win_x_left_low+win_x_left_high)/2
            right_bottom_x = (win_x_right_low+win_x_right_high)/2
            # print("left_bottom_x",left_bottom_x)
            # print("right_bottom_x",right_bottom_x)
        elif window==9:
            left_top_x = (win_x_left_low+win_x_left_high)/2
            right_top_x = (win_x_right_low+win_x_right_high)/2
            # print("left_top_x",left_top_x)
            # print("right_top_x",right_top_x)


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
    left_fit = np.polyfit(left_y, left_x, 2)  # 원래는 검출된 차선을 원본 이미지에 그리기 위해 피팅을 사용한 것으로 보임.
    right_fit = np.polyfit(right_y, right_x, 2)

    ploty = np.linspace(0, binary_img.shape[0] - 1, binary_img.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    out_img[non_zero_y[left_lane_indices], non_zero_x[left_lane_indices]] = [255, 255, 255]
    out_img[non_zero_y[right_lane_indices], non_zero_x[right_lane_indices]] = [255, 255, 255]


    return out_img, left_bottom_x, left_top_x, right_bottom_x, right_top_x
#, left_fit, right_fit
def histogram(frame):
    
    histogram_left = np.sum(frame[frame.shape[0]//2:,:frame.shape[1]//2], axis=0)
    histogram_right = np.sum(frame[frame.shape[0]//2:,frame.shape[1]//2:], axis=0)
    # print("left_x_base",histogram_left)
    # print("right_x_base",histogram_right)

    left_x_base = np.argmax(histogram_left)
    right_x_base = np.argmax(histogram_right)+frame.shape[1]//2
    # print("left_x_base",left_x_base)
    # print("right_x_base",right_x_base)

    return left_x_base,right_x_base

def image_filtering(warped_frame):
    
    #if 색상으로 차선 추출 hsv 사용
    hsv = cv2.cvtColor(warped_frame,cv2.COLOR_BGR2HSV)
    mask_yellow = cv2.inRange(hsv,(20,100,100),(40,255,255))
    mask_white = cv2.inRange(hsv,(0,0,200),(180,30,255))
    # cv2.imshow("yellow_mask",mask_yellow)
    # cv2.imshow("white_mask",mask_white)
    
    #노란색과 흰색 영역 합치기
    mask_lane = cv2.bitwise_or(mask_yellow, mask_white)
    cv2.imshow("sum_mask",mask_lane)
    blurred = cv2.GaussianBlur(mask_lane, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    cv2.imshow("edge_mask",edges)

    return mask_lane
    #if edge 검출해서 허프변환으로 차선 추출
    # gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray,(5,5),0)
    # edges = cv2.Canny(blur,50,150)
    # cv2.imshow("edges",edges)
    # print(edges.shape)
    # roi_vertices = [
        # (0, height),
        # (width // 2, height // 2),
        # (width, height)
    # ]

    # mask = np.zeros_like(edges)
    # cv2.fillPoly(mask, np.array([roi_vertices], np.int32), 255)
    # masked_edges = cv2.bitwise_and(edges, mask)

    # 허프 변환을 통한 차선 검출
    # lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=50)
    # return lines

# 주어진 원근 변환 행렬을 사용하여 이미지를 버드 아이뷰로 변환합니다.
def bird_eye_view_transform(frame, src_points, dst_points,height,width):
    M = cv2.getPerspectiveTransform(src_points,dst_points)
    warp_frame = cv2.warpPerspective(frame,M,(width,height))
    return warp_frame

#variables
# src_left_top =(260,720)
# src_left_bottom =(600,432)
# src_right_bottom = (640,432)
# src_right_top = (1200,720)

src_left_top =(580,460)
src_left_bottom =(203,720)
src_right_bottom = (1127,720)
src_right_top = (703,460)
default_angle = 0.5 #radian
k1 = 0.01
k2 = 0.01

src_points=np.float32([src_left_top,src_left_bottom,src_right_bottom,src_right_top])
dst_points=np.float32([(320, 0), (320, 720), (960, 720), (960, 0)])

while video.isOpened():
    # 프레임 읽기
    ret, frame = video.read()

    if not ret:
        break

    height,width = frame.shape[:2]    
    bird_eye_view = bird_eye_view_transform(frame,src_points,dst_points,height,width)
    bird_line_view = image_filtering(bird_eye_view)
    # print("image size", bird_line_view.shape)
    # 프레임 출력
    cv2.imshow("Video", frame)
    cv2.imshow("bird_eye_view",bird_eye_view)
    # cv2.imshow("bird_lane_view",bird_line_view)

    left_x_base,right_x_base = histogram(bird_line_view)
    # left_fit, right_fit = SlidingWindows(bird_line_view)
    out_image, left_bottom_x, left_top_x, right_bottom_x, right_top_x= SlidingWindows(bird_line_view)
    ref_distance = find_ref_distance(left_x_base,right_x_base)
    # print("ref_distance = ",ref_distance)
    ref_angle = find_ref_angle(left_bottom_x, left_top_x, right_bottom_x, right_top_x)
    # print("ref_angle = ",ref_angle)
    steer_angle = (k1*ref_distance)+(k2*ref_angle)-0.2
    print("steer_angle = ",steer_angle)
    # print("left_Theta = ",Theta_left + default_angle)
    # print("right_Theta = ",Theta_right + default_angle)
    if steer_angle>0.0 and steer_angle<1.0:
        steer_angle_msg = Float64()
        steer_angle_msg.data = steer_angle
        steer_angle_pub.publish(steer_angle_msg)
        rate.sleep()


    # result = draw_sliding_windows(bird_line_view,left_fit,right_fit,margin=100, height=height, n_win=10)
        # 'q' 키를 누르면 동영상 재생 종료
    cv2.imshow("Result",out_image)
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

    # 열려있는 창과 동영상 파일 객체 정리
video.release()
cv2.destroyAllWindows()