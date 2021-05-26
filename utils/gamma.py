import cv2
import os
import numpy as np
from shutil import copyfile

def gamma_transform(img, gamma):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:, :, 2] * gamma
    v_channel[v_channel > 255] = 255
    v_channel[v_channel < 0] = 0
    hsv[:, :, 2] = v_channel
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return img

img_dir = "./lab_image"
annotation_dir = "./lab_image_annotation"
for name in os.listdir(img_dir):
    img = cv2.imread(os.path.join(img_dir, name))
    dark = gamma_transform(img, 0.5)
    cv2.imwrite(os.path.join(img_dir, "darken_0_"+name), dark)
    sub_name = name.split(".")[0]
    copyfile(os.path.join(annotation_dir, sub_name+".xml"), os.path.join(annotation_dir, "darken_0_"+sub_name+".xml"))

    img = cv2.imread(os.path.join(img_dir, name))
    dark = gamma_transform(img, 0.7)
    cv2.imwrite(os.path.join(img_dir, "darken_1_"+name), dark)
    sub_name = name.split(".")[0]
    copyfile(os.path.join(annotation_dir, sub_name+".xml"), os.path.join(annotation_dir, "darken_1_"+sub_name+".xml"))

    img = cv2.imread(os.path.join(img_dir, name))
    dark = gamma_transform(img, 0.9)
    cv2.imwrite(os.path.join(img_dir, "darken_2_"+name), dark)
    sub_name = name.split(".")[0]
    copyfile(os.path.join(annotation_dir, sub_name+".xml"), os.path.join(annotation_dir, "darken_2_"+sub_name+".xml"))

    bright = gamma_transform(img, 1.4)
    cv2.imwrite(os.path.join(img_dir, "brighten_0_"+name), bright)
    sub_name = name.split(".")[0]
    copyfile(os.path.join(annotation_dir, sub_name+".xml"), os.path.join(annotation_dir, "brighten_0_"+sub_name+".xml"))

    bright = gamma_transform(img, 1.6)
    cv2.imwrite(os.path.join(img_dir, "brighten_1_"+name), bright)
    sub_name = name.split(".")[0]
    copyfile(os.path.join(annotation_dir, sub_name+".xml"), os.path.join(annotation_dir, "brighten_1_"+sub_name+".xml"))

    bright = gamma_transform(img, 1.8)
    cv2.imwrite(os.path.join(img_dir, "brighten_2_"+name), bright)
    sub_name = name.split(".")[0]
    copyfile(os.path.join(annotation_dir, sub_name+".xml"), os.path.join(annotation_dir, "brighten_2_"+sub_name+".xml"))


