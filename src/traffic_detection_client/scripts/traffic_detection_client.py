#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import socket
from std_msgs.msg import Int32
import threading
import time
import math

status_list = ["red_stop", "green_go", "yellow_back",
               "pedestrain_crossing", "speed_limited", "speed_unlimited"]


def parse_data(recv_data):
    if len(recv_data) == 0:
        return -1
    data = recv_data.split(',')
    if len(data) == 1:
        return int(data[0])
    elif len(data) > 1:
        #? what the msg is like here?
        if data[-1] == '':
            return int(data[-2])
        else:
            return int(data[-1])


def connect(socket, host, port):
    while True:
        try:
            print('try connect!')
            socket.connect((host, port))
            time.sleep(0.01)
            print('连接成功!')
            break
        except:
            pass


def talker():
    traffic_data_topic = rospy.get_param('~traffic_data_topic')
    traffic_data_pub = rospy.Publisher(traffic_data_topic, Int32, queue_size=10)

    # see configuration in index.py
    host = '192.168.2.111'
    port = 7777
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connect_thread = threading.Thread(
        target=connect, name='connect', args=(s, host, port))
    connect_thread.start()
    connect_thread.join()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        data = s.recv(1024)
        recv_data = parse_data(data)
        if recv_data == -1:
            break
        print(recv_data)
        if recv_data > 0:
            #? 如果同一张图片识别出多个box，那data不是应该有多个1..
            status_num = math.log(recv_data, 2)
            print(status_list[int(status_num)])
        # publish the status
        traffic_data_pub.publish(recv_data)
        rate.sleep()
    s.close()


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
