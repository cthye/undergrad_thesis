#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import os
import sys
import glob
import numpy as np
import math

# scale_factor = 10
class camera:
    def __init__(self):
        self.cap = cv2.VideoCapture('/dev/video0')
        # self.cap = cv2.VideoCapture('./1.avi')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        src_points = np.array([[19,114], [631,101], [19,407], [634,438]],dtype="float32")
        x = 680.
        y = 400. 
        scale_factor_x = x / 640.
        scale_factor_y = y / 480.
        scale_factor = max(scale_factor_x, scale_factor_y)
        dst_points = np.array([[0., 0.], [680, 0], [0, 400], [430, 400]], dtype="float32")
        dst_points = dst_points / scale_factor
        print(scale_factor, dst_points)
        self.M = cv2.getPerspectiveTransform(src_points, dst_points)

    def spin(self):
        ret, img = self.cap.read()
        n = 0
        while ret == True and cv2.waitKey(30) != 27:
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # kernel = np.ones((3,3), np.uint8)
            # gray_img = cv2.erode(gray_img, kernel, iterations=1)
            origin_thr = np.zeros_like(gray_img)
            origin_thr[(gray_img >= 125)] = 255
            
            cv2.imshow('b', origin_thr)
            binary_warped = cv2.warpPerspective(origin_thr, self.M, (640, 480), cv2.INTER_LINEAR)
            cv2.imshow('a', binary_warped)

            # cv2.imwrite('./warp_pic/' + str(n)+'.jpg', binary_warped)
            n += 1
            ret, img = self.cap.read()
            # if cv2.waitKey(0) == 27:
            #     break

if __name__ == '__main__':
    cam = camera()
    cam.spin()
