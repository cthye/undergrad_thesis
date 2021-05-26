#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def perspective_transform(img):
	"""
	Execute perspective transform
	"""
	img_size = (img.shape[1], img.shape[0])

	src=np.array([[157.11, 271.47], [10.75, 465.93], [479.51,272.32], [603.76, 470.34]], dtype=np.float32) # pixel, 图像坐标
	dst= np.array([[0., 0.], [276., 1264], [994, 0.], [610, 1264]], dtype=np.float32) # mm, 实际坐标

	scale_factor_x = 994 / 640.
	scale_factor_y = 1264 / 480.
	scale_factor = max(scale_factor_x, scale_factor_y)
	dst /= scale_factor

	m = cv2.getPerspectiveTransform(src, dst)
	m_inv = cv2.getPerspectiveTransform(dst, src)

	warped = cv2.warpPerspective(img, m, img_size, flags=cv2.INTER_LINEAR)
	unwarped = cv2.warpPerspective(warped, m_inv, (warped.shape[1], warped.shape[0]), flags=cv2.INTER_LINEAR)  # DEBUG

	return warped, unwarped, m, m_inv

def hls_select(img, channel='S', thresh=(90, 255)):
    # 1) Convert to HLS color space
    # 2) Apply a threshold to the S channel
    # 3) Return a binary image of threshold result
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    if channel == 'L':
        X = hls[:, :, 1]
    elif channel == 'H':
        X = hls[:, :, 0]
    elif channel == 'S':
        X = hls[:, :, 2]
    else:
        print('illegal channel !!!')
        return
    binary_output = np.zeros_like(X)
    binary_output[(X > thresh[0]) & (X <= thresh[1])] = 1
    return binary_output

def hsv_select(img, channel='S', thresh=(90, 255)):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    if channel == 'S':
        X = hsv[:, :, 1]
    elif channel == 'H':
        X = hsv[:, :, 0]
    elif channel == 'V':
        X = hsv[:, :, 2]
    else:
        print('illegal channel !!!')
        return
    binary_output = np.zeros_like(X)
    binary_output[(X >= thresh[0]) & (X <= thresh[1])] = 1
    return binary_output

def luv_select(img, channel='S', thresh=(90, 255)):
	luv = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
	if channel == 'L':
		X = luv[:, :, 0]
	elif channel == 'U':
		X = luv[:, :, 1]
	elif channel == 'V':
		X = luv[:, :, 2]
	else:
		print('illegal channel !!!')
		return
	binary_output = np.zeros_like(X)
	binary_output[(X > thresh[0]) & (X <= thresh[1])] = 1
	return binary_output

def r_select(img, thresh=(200, 255)):
    R = img[:,:,0]
    binary = np.zeros_like(R)
    binary[(R > thresh[0]) & (R <= thresh[1])] = 1
    return binary

def color_mask(hsv,low,high):
    # Return mask from HSV
    mask = cv2.inRange(hsv, low, high)
    return mask

def apply_color_mask(hsv,img,low,high):
    # Apply color mask to image
    mask = cv2.inRange(hsv, low, high)
    res = cv2.bitwise_and(img,img, mask= mask)
    return res

def apply_yellow_white_mask(img):
    image_HSV = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    # yellow_hsv_low  = np.array([ 0,  100,  100])
    # yellow_hsv_high = np.array([ 80, 255, 255])
    white_hsv_low  = np.array([ 0,   0,   160])
    white_hsv_high = np.array([ 255,  80, 255])
    # mask_yellow = color_mask(image_HSV,yellow_hsv_low,yellow_hsv_high)
    mask_white = color_mask(image_HSV,white_hsv_low,white_hsv_high)
    # mask_YW_image = cv2.bitwise_or(mask_yellow,mask_white)
    return mask_white

if __name__ == '__main__':
	# img_file = '/home/album/autocar_ws/utils/hilens/green/frame_102.jpg'
	img_file = './pic/0.jpg'
	wrap_img = mpimg.imread(img_file)# rgb
	# wrap_img = cv2.imread(img_file)
	#wrap_img = wrap_img[350:600, 400:900]
	#wrap_img = cv2.medianBlur(wrap_img, 5)
	hsv = cv2.cvtColor(wrap_img, cv2.COLOR_RGB2HSV)
	# 绿色
	# h_binary = hsv_select(wrap_img, channel='H', thresh=(60, 90))
	# s_binary = hsv_select(wrap_img, channel='S', thresh=(100, 255))
	# v_binary = hsv_select(wrap_img, channel='V', thresh=(40, 255))
	# 红色
	h_binary = hsv_select(wrap_img, channel='H', thresh=(0, 100))
	s_binary = hsv_select(wrap_img, channel='S', thresh=(0, 20))
	v_binary = hsv_select(wrap_img, channel='V', thresh=(200, 255))
	f, axs = plt.subplots(2, 3, figsize=(16, 9))
	f.tight_layout()
	origin =  np.zeros_like(h_binary)
	origin[(h_binary != 0) & (s_binary != 0) & (v_binary != 0)] = 1
	axs[0, 0].imshow(origin, cmap='gray')
	axs[0, 0].set_title('Original Image', fontsize=18)
	axs[0, 1].imshow(h_binary, cmap='gray')
	axs[0, 1].set_title('H channal filter', fontsize=18)
	axs[1, 0].imshow(s_binary, cmap='gray')
	axs[1, 0].set_title('S channal filter', fontsize=18)
	axs[1, 1].imshow(v_binary, cmap='gray')
	axs[1, 1].set_title('V channal filter', fontsize=18)
	axs[0, 2].imshow(hsv)
	axs[0, 2].set_title('hsv', fontsize=18)
	axs[1, 2].imshow(wrap_img)
	axs[1, 2].set_title('wrap_img', fontsize=18)
	plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
	plt.show()

	# luv = cv2.cvtColor(wrap_img, cv2.COLOR_BGR2LUV)
	# l_binary = luv_select(wrap_img, channel='L', thresh=(200, 255))
	# u_binary = luv_select(wrap_img, channel='U', thresh=(90, 110))
	# v_binary = luv_select(wrap_img, channel='V', thresh=(130, 150))
	# f, axs = plt.subplots(2, 3, figsize=(16, 9))
	# f.tight_layout()
	# origin =  np.zeros_like(l_binary)
	# origin[(l_binary != 0) & (u_binary != 0) & (v_binary != 0)] = 1
	# axs[0, 0].imshow(origin, cmap='gray')
	# axs[0, 0].set_title('Original Image', fontsize=18)
	# axs[0, 1].imshow(l_binary, cmap='gray')
	# axs[0, 1].set_title('L channal filter', fontsize=18)
	# axs[1, 0].imshow(u_binary, cmap='gray')
	# axs[1, 0].set_title('U channal filter', fontsize=18)
	# axs[1, 1].imshow(v_binary, cmap='gray')
	# axs[1, 1].set_title('V channal filter', fontsize=18)
	# axs[0, 2].imshow(luv)
	# axs[0, 2].set_title('luv', fontsize=18)
	# plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
	# plt.show()

	# wrap_img, unwarped, m, m_inv = perspective_transform(img)
	# plt.imshow(wrap_img)
	# plt.show()

	# plt.imshow(unwarped)
	# plt.show()

	if False:
		cap = cv2.VideoCapture('/home/album/autocar_ws/utils/turn_right.avi')
		if not cap.isOpened():
			print('open fail')
			exit(-1)

		while cv2.waitKey(30) != 27:
			_, wrap_img = cap.read()
			hsv = cv2.cvtColor(wrap_img, cv2.COLOR_BGR2HSV)
			h_binary = hsv_select(wrap_img, channel='H', thresh=(0, 120))
			s_binary = hsv_select(wrap_img, channel='S', thresh=(0, 70))
			v_binary = hsv_select(wrap_img, channel='V', thresh=(170, 255))
			cv2.imshow('hsv', hsv)

			if cv2.waitKey(30) == ord('s'):
				f, axs = plt.subplots(2, 3, figsize=(16, 9))
				f.tight_layout()
				origin =  np.zeros_like(h_binary)
				origin[(h_binary != 0) & (s_binary != 0) & (v_binary != 0)] = 1
				axs[0, 0].imshow(origin, cmap='gray')
				axs[0, 0].set_title('Original Image', fontsize=18)
				axs[0, 1].imshow(h_binary, cmap='gray')
				axs[0, 1].set_title('H channal filter', fontsize=18)
				axs[1, 0].imshow(s_binary, cmap='gray')
				axs[1, 0].set_title('S channal filter', fontsize=18)
				axs[1, 1].imshow(v_binary, cmap='gray')
				axs[1, 1].set_title('V channal filter', fontsize=18)
				axs[0, 2].imshow(hsv)
				axs[0, 2].set_title('hsv', fontsize=18)
				plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
				plt.show()
