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
            print("Sending: " + message)
            while not rospy.is_shutdown():
                msg.data = message
                # print(message)
                self.pub.publish(msg)
                rate.sleep()
                n = n+1
                if n>100:
                    return
        else:
            print("Sending: " + message)
            while not rospy.is_shutdown():
                msg.data = message
                # print(message)
                self.pub.publish(msg)
                rate.sleep()
                if self.received:
                    self.received = False
                    return


    def root_callback(self, msg):
        # print(msg.data)
        if msg.data == 'received':
            self.received = True
        elif msg.data == 'getOs':
            print("Received: " + msg.data)
            self.sendRequest('received')
            # self.camera.start_preview()
            # time.sleep(5)
            self.camera.capture('/home/pi/Desktop/testExample.jpg')
            # self.camera.stop_preview()
        else:
            print("Received: " + msg.data)



if __name__ == '__main__':
    camera = Camera()
    rospy.spin()
