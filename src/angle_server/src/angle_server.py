#! /usr/bin/env python
import rospy
from gevent.server import StreamServer
from mprpc import RPCServer
from geometry_msgs.msg import Twist

cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
cam_cmd = Twist()
k = 1

class orderServer(RPCServer):
    def order(self, a, s):
        angle = float(a)
        score = float(s)
        print("angle:" + str(angle) + "  score:" + str(score))
        if score > 0.9:
            cam_cmd.linear.x = 0.1
            cam_cmd.angular.z = k * angle
            cmd_pub.publish(cam_cmd)
        else:
            cam_cmd.linear.x = 0
            cam_cmd.angular.z = 0
            cmd_pub.publish(cam_cmd)


print("?????")
if __name__ == '__main__':
    rospy.init_node('angle_server',anonymous=True)
    #rate = rospy.Rate(30)
    print("tracking start......................")
    server = StreamServer(('0.0.0.0',6000),orderServer())
    server.serve_forever()


