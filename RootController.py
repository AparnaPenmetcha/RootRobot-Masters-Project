import gatt
import threading
import time,termios,tty,sys
import datetime
import rospy
from std_msgs.msg import String
import requests


class RootController:
    
    def __init__(self):
        rospy.init_node('rootPi', anonymous=True)
        rospy.Subscriber('toRoot', String, self.root_callback)
        self.pub = rospy.Publisher('fromRoot', String, queue_size=10)
        self.received = False

    def sendRequest(self, message):

        # rospy.loginfo('Sending to Camera: %s', messageString.data)
        msg = String()
        rate = rospy.Rate(1)  # update rate in Hz
        if message == 'received':
            n = 0
            rospy.loginfo("Sending: %s", msg.data)
            while not rospy.is_shutdown():
                msg.data = message
                self.pub.publish(msg)
                n = n + 1
                if n > 100:
                    return
        else:
            rospy.loginfo("Sending: %s", msg.data)
            while not rospy.is_shutdown():
                msg.data = message
                self.pub.publish(msg)
                if self.received:
                    self.received = False
                    return

    def root_callback(self, msg):
        if msg.data == 'received':
            self.received = True
        else:
            rospy.loginfo("Received: %s", msg.data)
            self.sendRequest('received')


if __name__ == '__main__':

    rootController = RootController()
    rootController.sendRequest('getOs')
    

    rospy.spin()
        
    