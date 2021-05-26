#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import os
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
# import cProfile
# import line_profiler
# import sys
# ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
# if os.path.exists(ros_path):
#     sys.path.remove(ros_path)
import cv2


class LaneDetector:
    def __init__(self):
        # 调试选项
        self.debug_viz = rospy.get_param('~debug_viz')
        self.viz_origin = rospy.get_param('~viz_origin')
        self.viz_fit = rospy.get_param('~viz_fit')
        self.viz_preprocess  = rospy.get_param('~viz_preprocess')
        self.viz_target_point = rospy.get_param('~viz_target_point')

        # 控制参数
        self.k_left = rospy.get_param('~k_left')
        self.k_right = rospy.get_param('~k_right')
        self.y_offset = rospy.get_param('~y_offset')
        self.turn_thres = rospy.get_param('~turn_thres')
        self.step_threshold=rospy.get_param('~step_threshold')
        self.accmulated_not_step_frames=rospy.get_param("~accmulated_not_step_frames")
        # 物理参数
        self.road_width = rospy.get_param('~road_width')
        self.wheel_base = rospy.get_param('~wheel_base')
        self.dist_cv = rospy.get_param('~dist_cv')

        # 话题
        cmd_topic = rospy.get_param('~cmd_topic')
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        # 相机参数
        video_path = rospy.get_param('~video_path')
        self.frame_width = rospy.get_param('~frame_width')
        self.frame_height = rospy.get_param('~frame_height')
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            rospy.logfatal('Open videoCapture failed!')
            exit(-1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cv2.VideoWriter_fourcc(*("MJPG")))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        # 透视变换参数
        self.turn_left_mid_x = rospy.get_param('~turn_left_mid_x')
        self.turn_right_mid_x = rospy.get_param('~turn_right_mid_x')
        src_points = np.array(rospy.get_param('~src_points'), dtype=np.float32)
        dst_points = np.array(rospy.get_param('~dst_points'), dtype=np.float32)
        x_real = rospy.get_param('~x_real')
        y_real = rospy.get_param('~y_real')
        self.x_mm_per_pixel = rospy.get_param('~x_mm_per_pixel')
        self.y_mm_per_pixel = rospy.get_param('~y_mm_per_pixel')
        scale_factor_x = x_real / self.frame_width
        scale_factor_y = y_real / self.frame_height
        scale_factor = max(scale_factor_x, scale_factor_y)
        dst_points /= scale_factor
        self.warp_mat = cv2.getPerspectiveTransform(src_points, dst_points)
        self.img=np.zeros((self.frame_height,self.frame_width,3),dtype=np.uint8)
        # 掩膜参数
        self.use_mask = rospy.get_param('~use_mask')
        mask_ratio = rospy.get_param('~mask_ratio')
        self.mask = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        self.mask[int(self.mask.shape[0] * mask_ratio):,  :] = (255, 255, 255)

        # 预处理参数
        self.use_gray = bool(rospy.get_param('~use_gray'))
        self.use_otsu = bool(rospy.get_param('~use_otsu'))
        self.gray_threshold = rospy.get_param('~gray_threshold')

        self.use_hsv = bool(rospy.get_param('~use_hsv'))
        self.hsv_h_threshold = rospy.get_param('~hsv_h_threshold')
        self.hsv_s_threshold = rospy.get_param('~hsv_s_threshold')
        self.hsv_v_threshold = rospy.get_param('~hsv_v_threshold')

        # 曲线拟合参数
        self.hist_x_ratio = rospy.get_param('~hist_x_ratio')
        self.hist_y_ratio = rospy.get_param('~hist_y_ratio')

        self.n_windows = rospy.get_param('~n_windows')
        self.min_windows = rospy.get_param('~min_windows')
        self.min_pixels = rospy.get_param('~min_pixels')
        self.window_width = rospy.get_param('~window_width')
        self.window_height = rospy.get_param('~window_height')

        self.turn_left_road_width_ratio = rospy.get_param('~turn_left_road_width_ratio')
        self.turn_right_road_width_ratio = rospy.get_param('~turn_right_road_width_ratio')
        self.lane_y_ratio = rospy.get_param('~lane_y_ratio')
        self.turn_y_ratio = rospy.get_param('~turn_y_ratio')
        self.inside_target_y_ratio = rospy.get_param('~inside_target_y_ratio')
        self.outer_target_y_ratio = rospy.get_param('~outer_target_y_ratio')

        self.speed = rospy.get_param('~speed')

        # self.is_initial = True
        # self.target_lane = "outside"
        # self.target_LorR = -1
        self.correct_detect_time = 0
        self.aP = [0.0, 0.0]
        # self.Timer = 0
        self.cam_cmd = Twist()
        #计算瞄点变化
        self.last_aP=[0.0,0.0]
        self.is_step=False
        self.begin_run=True
        self.accmulated_not_step=0
        self.is_ok=True
        self.map1,self.map2=self.initialize_perspective_to_maps()

    def initialize_perspective_to_maps(self):
        warp_mat=self.warp_mat.astype(np.float32)
        inv_perspective=np.linalg.inv(warp_mat)
        img_height=self.img.shape[0]
        img_width=self.img.shape[1]
        xy=np.zeros((img_height,img_width,2),dtype=np.float32)
        for y  in range(img_height):
            for x in range(img_width):
                xy[y,x,0]=x
                xy[y,x,1]=y
        map_transformed=cv2.perspectiveTransform(xy,inv_perspective)
        map1,map2=cv2.convertMaps(map_transformed[:,:,0],map_transformed[:,:,1],cv2.CV_16SC2)
        return map1,map2


    def spin(self):
        ret, img = self.cap.read()
        self.img=img
        if ret:
            if self.viz_origin:
                cv2.imshow('origin', img)
            begin = time.time()
            binary_warped = self.binaryzation(img)
            binary_finished = time.time()
            print("binaryzation--- %s seconds ---" % (binary_finished - begin))
            # profile = line_profiler.LineProfiler(self.fitLine)
            # profile.enable()
            tmp = self.fitLine(binary_warped)
            fitline_finished = time.time()
            print("fitline--- %s seconds ---" % (fitline_finished - binary_finished))
            # profile.disable()
            # profile.print_stats(sys.stdout)
            if tmp is None:
                self.cam_cmd.angular.y = 1
                self.cmd_pub.publish(self.cam_cmd)
                self.cam_cmd.angular.y = 0
            else:
                model, pixelX, pixelY, good_window_num = tmp
                if good_window_num <= 4:
                    self.cam_cmd.angular.y = 1
                    self.cmd_pub.publish(self.cam_cmd)
                    self.cam_cmd.angular.y = 0
                    return
                self.calcSteer(binary_warped, model, pixelX, pixelY)
            cal_finished = time.time()
            print("calcSteer--- %s seconds ---" % (cal_finished - fitline_finished))
            print('------------------------------total time-------------------------',time.time()-begin)
            if self.debug_viz:
                if cv2.waitKey(1) == ord('s'):
                    cv2.waitKey(0)
        elif ret and self.is_low_encounter_pedestrains==True:
            return

    def binaryzation(self, img):
        if self.use_mask:
            img = cv2.bitwise_and(img, self.mask)

        # 灰度阈值
        img = cv2.medianBlur(img, 5)
        if self.use_gray:
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # img pre-process
            gray_img = cv2.equalizeHist(gray_img)
            origin_thr = np.zeros_like(gray_img)
            if self.use_otsu:
                _, origin_thr = cv2.threshold(gray_img, 0, 255, cv2.THRESH_OTSU)
            else:
                origin_thr[(gray_img >= self.gray_threshold[0]) & (gray_img <= self.gray_threshold[1])] = 255
        # HSV阈值
        if self.use_hsv:
            hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            hsv_binary=cv2.inRange(hsv_img,(self.hsv_h_threshold[0],self.hsv_s_threshold[0],self.hsv_v_threshold[0]),( self.hsv_h_threshold[1], self.hsv_s_threshold[1],self.hsv_v_threshold[1]))
        # 组合
        # combined = np.zeros_like(origin_thr)
        # combined[(hsv_binary == 255) & (origin_thr == 255)] = 255
        combined = cv2.bitwise_and(hsv_binary, origin_thr)
        # 透视变换
        binary_warped=cv2.remap(combined,self.map1,self.map2,cv2.INTER_LINEAR)
        binary_warped=cv2.GaussianBlur(binary_warped,(7,7),0)
        if self.viz_preprocess and self.debug_viz:
            cv2.imshow('gray', origin_thr)
            cv2.imshow('equalizaHist', gray_img)
            cv2.imshow('hsv', hsv_binary)
            cv2.imshow('preprocess', combined)
        return binary_warped

    def fitLine(self, binary_warped):
        hist_x = np.sum(binary_warped[int(self.hist_y_ratio*binary_warped.shape[1]):, :int(self.hist_x_ratio*binary_warped.shape[0])], axis=0) # 只看下半部分
        lane_base = np.argmax(hist_x)

        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        lane_current = lane_base
        margin = self.window_width
        minpix = self.min_pixels
        lane_inds = []
        good_window_num = 0
        for window in range(self.n_windows):
            win_y_low = binary_warped.shape[0] - (window + 1) * self.window_height
            win_y_high = binary_warped.shape[0] - window * self.window_height
            win_x_low = lane_current - margin
            win_x_high = lane_current + margin
            if self.viz_fit and self.debug_viz:
                if len(binary_warped.shape) < 3:
                    binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
                cv2.rectangle(binary_warped, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0 ,0), 3)

            pixel_common = ((nonzeroy >= win_y_low) & (nonzerox >= win_x_low) & (nonzerox < win_x_high))
            good_inds = (pixel_common & (nonzeroy < win_y_high)) .nonzero()[0]
            high_inds= (pixel_common & (nonzeroy < (win_y_low+5))).nonzero()[0]
            if len(good_inds) > minpix:
                good_window_num += 1
                lane_inds.append(good_inds)
                if   len(high_inds)>0:
                    lane_current=int(np.mean(nonzerox[high_inds]))
                else:
                    lane_current=int(np.mean(nonzerox[good_inds]))

            elif window >= self.min_windows:
                break

        pixelX = np.array([])
        pixelY = np.array([])
        if len(lane_inds) > 0:
            lane_inds = np.concatenate(lane_inds)
            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

        if (pixelX.size == 0):
            return None

        model = np.polyfit(pixelY,pixelX, 2)

        if self.viz_fit and self.debug_viz:
            x = np.polyval(model, pixelY)
            draw_points = (np.asarray([x, pixelY]).T).astype(np.int32)   # needs to be int32 and transposed
            cv2.polylines(binary_warped, [draw_points], False, (0,0,255), 3)  # args: image, points, closed, color
            cv2.imshow('fit_line', binary_warped)

        return model, pixelX, pixelY, good_window_num


    def calcSteer(self, binary_warped, model, pixelX, pixelY):
        sorted_pixel_y = np.argsort(pixelY)
        aim_y = sorted_pixel_y[int(len(pixelY) * self.lane_y_ratio) - 1]  # 区分左右车道线的瞄点
        aim_point = [pixelX[aim_y], pixelY[aim_y]]
        # 计算aimLaneP处斜率，从而得到目标点的像素坐标
        a, b, c = model
        slope_of_aim_point = 2 * a * aim_point[1] + b  # 斜率
        y_target = binary_warped.shape[0] + (self.dist_cv - self.wheel_base) / self.y_mm_per_pixel
        # x = k * (y - y0) + x0，瞄点的x坐标
        x_intertcept = slope_of_aim_point * (y_target - aim_point[1]) + aim_point[0]

        aim_y = sorted_pixel_y[int(len(pixelY) * self.turn_y_ratio)]  # 区分转向的瞄点
        aim_point = [pixelX[aim_y], pixelY[aim_y]]
        slope_of_aim_point = 2 * a * aim_point[1] + b
        mid_x = self.turn_left_mid_x
        if slope_of_aim_point > 0:  # 判断转向
            mid_x = self.turn_left_mid_x
        else:
            mid_x = self.turn_right_mid_x

        if (x_intertcept > mid_x):
            LorR = -1  # 识别到右车道线，则目标点要在右车道线的左侧
        else:
            LorR = 1  # 识别到左车道线，则目标点要在左车道线右侧

        lane_criteria = slope_of_aim_point * LorR
        if lane_criteria < 0: # 外道
            lane = "outside"
        else:
            lane = "inside"

        # if self.is_initial:
        #     if lane==self.target_lane and LorR==self.target_LorR:
        #         self.correct_detect_time += 1
        #     else:
        #         if aim_point[0] < mid_x * 0.9:
        #             self.is_initial = True

        #         lane = self.target_lane
        #         LorR = self.target_LorR
        #         self.correct_detect_time=0
        #     # 连续正确识别到进入状态，结束初始化
        #     if self.correct_detect_time>4:
        #         self.is_initial = False
        #         print('initialize finish')

        #     # 尽量贴线
        #     LorR *= 0.475

        if (abs(slope_of_aim_point) > self.turn_thres):  # 使弯道目标点更靠近路中心
            if slope_of_aim_point > 0: # 左拐
                LorR *= self.turn_left_road_width_ratio
            else:
                LorR *= self.turn_right_road_width_ratio

        if lane == "outside":
            target_y = sorted_pixel_y[int(len(pixelY) * self.outer_target_y_ratio)]
        else:
            target_y = sorted_pixel_y[int(len(pixelY) * self.inside_target_y_ratio)]

        target_point = [pixelX[target_y], pixelY[target_y]]  # 用于计算驾驶角度的目标点
        slope_of_target_point = 2 * a * target_point[1] + b
        theta = math.atan(slope_of_target_point)
        x = target_point[0] + (LorR) * self.road_width / 2 * math.cos(theta) * math.cos(theta)
        y = target_point[1] - (LorR) * self.road_width / 2 * math.cos(theta) * math.sin(theta)

        # 计算目标点的真实坐标
        self.aP[0] = (x - mid_x) * self.x_mm_per_pixel
        self.aP[1] = (binary_warped.shape[0] - y) * self.y_mm_per_pixel + self. y_offset
        steer_angle = math.atan(2 * self.wheel_base * self.aP[0] / (self.aP[0] * self.aP[0] + (self.aP[1] + self.dist_cv) * (self.aP[1] + self.dist_cv)))
        #增加控制稳定性,利用目标点跳变判断
        ap_changed_distance=0
        if self.begin_run==True:
            self.begin_run=False
        else:
            ap_changed_distance=np.sqrt((self.aP[1]-self.last_aP[1])**2+(self.aP[0]-self.last_aP[0])**2)
            if ap_changed_distance>self.step_threshold and self.is_step==False:
                self.is_step=True
                self.is_ok=False
            elif ap_changed_distance>self.step_threshold and self.is_step==True:
                self.accmulated_not_step=0
            elif ap_changed_distance<self.step_threshold and self.is_ok==False:
                self.accmulated_not_step+=1
            if self.accmulated_not_step>= self.accmulated_not_step_frames:
                self.is_step=False
                self.is_ok=True
                self.accmulated_not_step=0
        self.last_aP=self.aP[:]
        if self.is_ok==True:
            if steer_angle < 0:
                self.cam_cmd.angular.z = self.k_left * steer_angle
            else:
                self.cam_cmd.angular.z = self.k_right * steer_angle

        # if self.is_initial:
        #     self.cam_cmd.linear.z = 1
        self.cam_cmd.linear.x = self.speed
        self.cmd_pub.publish(self.cam_cmd)
        # if self.is_initial:
        #     self.cam_cmd.linear.z = 0

        # 调试使用，可视化状态
        if self.viz_target_point and self.debug_viz:
            if len(binary_warped.shape) < 3:
                binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
            cv2.circle(binary_warped, center=(int(target_point[0]), int(
                target_point[1])), radius=10, color=(255, 0, 0), thickness=-1)
            cv2.circle(binary_warped, center=(int(x), int(
                y)), radius=10, color=(0, 255, 0), thickness=-1)
            cv2.line(binary_warped, (int(mid_x), 0),
                     (int(mid_x), 480), (255, 255, 0))
            cv2.putText(binary_warped, 'slope: ' + str(slope_of_aim_point),
                        (40, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(binary_warped, 'lane: ' + str(lane), (40, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(binary_warped, 'x_intertcept: ' + str(x_intertcept),
                        (40, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(binary_warped, 'steer_angle: ' + str(self.cam_cmd.angular.z),
                        (40, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(binary_warped, 'LorR: ' + str(LorR),
                        (40, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(binary_warped, 'current_status: stable' if self.is_ok else 'current_status:unstable', (40, 220), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0,255), 2, cv2.LINE_AA)
            cv2.imshow('target_point', binary_warped)

if __name__ == '__main__':
    # plt.ion()
    rospy.init_node('lane_detection', anonymous=True)
    rate = rospy.Rate(30)

    lane_detector = LaneDetector()
    while not rospy.is_shutdown():
        # cProfile.run('lane_detector.spin()', sort="cumulative")
        lane_detector.spin()
        rate.sleep()
