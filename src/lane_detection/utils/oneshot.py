#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import time 

cap = cv2.VideoCapture('/dev/video0')
ret=cap.set(3,640)
ret=cap.set(4,480)
if not cap.isOpened():
    print('open camera failed.')
    exit(-1)

n = 0
while True:
    ret, frame = cap.read()
    cv2.imshow('img', frame)
    key = cv2.waitKey(30)
    if  key == ord('s'):
        cv2.imwrite('./pic/' + str(n) + '.jpg', frame)
        n += 1
        print('take pic')
    elif key == 27:
        break

cap.release()