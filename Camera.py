# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy
from std_msgs.msg import String



class Camera:

    def __init__(self):
        rospy.init_node('RootTest', anonymous='True')

        # Prepare publisher on the 'sendRoot' topic
        self.pub = rospy.Publisher('toRoot', String, queue_size=10)
        rospy.Subscriber('fromRoot', String, self.root_callback)
        self.received = False
        self.camera = PiCamera()


    def sendRequest(self, message):

        # rospy.loginfo('Sending to Camera: %s', messageString.data)
        msg = String()
        rate = rospy.Rate(1)  # update rate in Hz
        if message == 'received':
            n = 0
            while not rospy.is_shutdown():
                msg.data = message
                rospy.loginfo('%s(%.2f) %s' % (rospy.get_name(), rospy.get_time(), message))
                self.pub.publish(msg)
                n = n+1
                if n>100:
                    return
        else:
            while not rospy.is_shutdown():
                msg.data = message
                rospy.loginfo('%s(%.2f) %s' % (rospy.get_name(), rospy.get_time(), message))
                self.pub.publish(msg)
                if self.received:
                    self.received = False
                    return

        print('checkpoint7')

    def root_callback(self, msg):

            # print('checkpoint5')
            self.sendRequest('received')
            # camera.start_preview()
            print(msg.data)
            # time.sleep(5)
            # camera.capture('/home/pi/Desktop/testExample.jpg')
            # camera.stop_preview()



if __name__ == '__main__':
    camera = Camera()
    rospy.spin()
