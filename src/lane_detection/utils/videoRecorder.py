#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import time 

cap = cv2.VideoCapture('/dev/video0')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print('open camera failed.')
    exit(-1)

# fps = cap.get(cv2.CAP_PROP_FPS)

sz = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

video_writer = cv2.VideoWriter('./record_'+ str(time.time()) + '.avi', cv2.VideoWriter_fourcc('I', '4', '2', '0'), 30, sz)

while cv2.waitKey(30) != 27:
    ret, frame = cap.read()
    video_writer.write(frame)
    cv2.imshow('img', frame)

cap.release()