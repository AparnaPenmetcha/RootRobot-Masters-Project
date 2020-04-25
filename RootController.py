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
        rospy.spin()
    
    def sendRequest(self, message):
        
        messageString = String()
        messageString.data = message
    
        rospy.loginfo('Sending to Camera: %s', messageString.data)
        self.pub.publish(messageString)
        
    def root_callback(self,msg):
        rospy.loginfo('I heard %s.', msg.data)


if __name__ == '__main__':
 
    rootController = RootController()
    rootController.sendRequest('')


        
        
    