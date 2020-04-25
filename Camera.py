# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy
from std_msgs.msg import String

def getOLocation(im):

    locations = []

    imgwidth=im.shape[0]
    imgheight=im.shape[1]


    y1 = 0
    M = imgwidth//3
    N = imgheight//3


    n = 0
    for x in range(0,imgwidth,M):
        for y in range(0, imgheight, N):
            x1 = x + M
            y1 = y + N
            tiles = im[x:x+M,y:y+N]

            cv2.rectangle(im, (x, y), (x1, y1), (0, 255, 0))
            # cv2.imwrite(r'/Users/shaivpatel/Desktop/Apoopoo/tictactoe/after/' +str(x)+ str(y)+"SAMPLE.png",tiles)

            # Read image.

            gray = cv2.cvtColor(tiles, cv2.COLOR_BGR2GRAY)

            gray = cv2.medianBlur(gray, 5)

            rows = gray.shape[0]
            detected_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                      param1=100, param2=30,
                                      minRadius=1, maxRadius=3000)

            # Draw circles that are detected.
            if detected_circles is not None:
                locations.append(n)

            n = n + 1
    print(locations)
    return locations

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
                if n>10:
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
            self.camera.capture('new.jpg')
            # self.camera.stop_preview()

            x = 85
            y = 40
            h = 1000
            w = 1000

            im1 = cv2.imread(r'old.jpg')
            im1 = im1[y:y + h, x:x + w]
            im1 = cv2.resize(im1, (600, 600))
            im2 = cv2.imread(r'new.jpg')
            im2 = im2[y:y + h, x:x + w]
            im2 = cv2.resize(im2, (600, 600))

            oldLocations = getOLocation(im1)
            newLocations = getOLocation(im2)

            newLocation = -1
            for loc in newLocations:
                if loc not in oldLocations:
                    print('{} is a new location'.format(loc))
                    newLocation = loc

            if newLocation > -1:
                cv2.imwrite('/home/pi/Desktop/old.jpg', im2)
                self.sendRequest(str(newLocation))

        else:
            print("Received: " + msg.data)



if __name__ == '__main__':
    camera = Camera()
    # location = 3
    # camera.sendRequest(str(location))
    rospy.spin()
