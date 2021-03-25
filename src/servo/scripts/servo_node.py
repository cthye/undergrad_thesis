#!/usr/bin/python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from enum import Enum

State = Enum('State', ('STOP', 'LANE_CONTROL'))

class ControllerManager:
    def __init__(self):
        self.getParam()

        self.lane_cmd_sub = rospy.Subscriber(
            self.lane_cmd_topic, Twist, self.laneCmdCallback, queue_size=1)
        self.traffic_data_sub = rospy.Subscriber(
            self.traffic_data_topic, Int32, self.trafficDataCallback, queue_size=1)
        self.cmd_pub = rospy.Publisher(
            self.cmd_topic, Twist, queue_size=1)

        self.vel_msg = Twist()
        self.state = State.LANE_CONTROL
        self.is_speed_limited = False

    def getParam(self):
        # msg topics
        self.lane_cmd_topic = rospy.get_param('~lane_cmd_topic')
        self.cmd_topic = rospy.get_param('~cmd_topic')
        self.traffic_data_topic = rospy.get_param('~traffic_data_topic')

        # speed
        self.speed_adjust_step = rospy.get_param('~speed_adjust_step')
        self.speed_lowerbound = rospy.get_param('~speed_lowerbound')
        self.speed_upperbound = rospy.get_param('~speed_upperbound')

    def laneCmdCallback(self, msg):
        #todo: control the car when lane is missing
        # if msg.angular.y > 0.5:
        #     rospy.loginfo('lost lane')
        #     # lane missing, stop it
        #     self.vel_msg.linear.x = 0
        # else:
        if self.state == State.LANE_CONTROL and not self.is_speed_limited:
            self.vel_msg = msg
        
        # always keep sending angular z
        else:
            self.vel_msg.angular.z = msg.angular.z
    
    def trafficDataCallback(self, msg):
        '''
        0: red_stop
        1: green_go
        2: yellow_back
        3: pedestrain_crossing
        4: speed_limited
        5: speed_unlimited
        '''
        #TODO: publish class lable directly in traffic_detection_client.py
        red_stop = msg.data & 1
        green_go = (msg.data >> 1) & 1
        yellow_back = (msg.data >> 2) & 1
        pedestrain_crossing = (msg.data >> 3) & 1
        speed_limited = (msg.data >> 4) & 1
        speed_unlimited = (msg.data >> 5) & 1

        if red_stop:
            self.state = State.STOP
        elif green_go or yellow_back:
            self.state = State.LANE_CONTROL
        elif speed_limited:
            self.is_speed_limited = True
        elif speed_unlimited:
            self.is_speed_limited = False
    
    def adjust_speed(self):
        print(self.state, self.vel_msg.linear.x)
        if self.state == State.STOP:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        elif self.is_speed_limited:
            print("speed down")
            self.vel_msg.linear.x = max(self.vel_msg.linear.x - self.speed_adjust_step, self.speed_lowerbound)
        elif not self.is_speed_limited:
            print("speed up")
            self.vel_msg.linear.x = min(self.vel_msg.linear.x + self.speed_adjust_step, self.speed_upperbound)

    def spin(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            self.adjust_speed()
            self.cmd_pub.publish(self.vel_msg)
            rate.sleep()

    def on_stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.cmd_pub.publish(self.vel_msg)

if __name__ == '__main__':
    rospy.init_node('servo')
    cm = ControllerManager()
    rospy.on_shutdown(cm.on_stop)
    cm.spin()
