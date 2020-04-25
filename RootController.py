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
        print('checkpoint2')

        print('checkpoint3')
    
    def sendRequest(self, message):
        print('checkpoint5')
        messageString = String()
        messageString.data = message
        print('checkpoint6')

        rospy.loginfo('Sending to Camera: %s', messageString.data)
        self.pub.publish(messageString)
        print('checkpoint7')

    def root_callback(self,msg):
        rospy.loginfo('I heard %s.', msg.data)


if __name__ == '__main__':

    print('checkpoint1')
    rootController = RootController()
    print('checkpoint4')
    rootController.sendRequest('')
    print('checkpoint8')

    rospy.spin()
        
    