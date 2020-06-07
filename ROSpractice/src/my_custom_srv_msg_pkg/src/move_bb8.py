#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class MoveBB8():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel',
            Twist,queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)
    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections>0:
                self.pub.publish(self.cmd)
                rospy.loginfo('Cmd Published')
                break
            else:
                self.rate.sleep()
    def shutdownhook(self):
        self.stop_bb8()
        self.ctrl_c = True
    def stop_bb8(self):
        rospy.loginfo('shutdown.')
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.publish_once_in_cmd_vel()
    def move_bb8(self,time,linear_speed=.2,angular_speed=.2):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0
        rospy.loginfo('Moving BB8')
        while self.ctrl_c and i <= time:
            self.publish_once_in_cmd_vel()
            i+=1
            self.rate.sleep()
        self.stop_bb8()


if __name__ == '__main__':
    rospy.init_node('testing',anonymous=True)
    obj = MoveBB8()
    try:
        obj.move_bb8()
    except rospy.ROSInterruptException:
        pass