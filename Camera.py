# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy
from std_msgs.msg import String

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)



def root_callback(msg):


    camera.start_preview()
    print('preview starting')
    time.sleep(5)
    camera.capture('/home/pi/Desktop/testExample.jpg')
    camera.stop_preview()

def main():
    """Node setup and main ROS loop"""
    rospy.init_node('RootTest', anonymous='True')

    #Prepare publisher on the 'sendRoot' topic
    pub = rospy.Publisher('toRoot', String, queue_size=10)
    rospy.Subscriber('fromRoot', String, root_callback)   # Use the 'sendRoot' topic
    # msg = String()
    # rate = rospy.Rate(1)  #update rate in Hz
    # command=['red','green','blue']


if __name__ == '__main__':
    main()