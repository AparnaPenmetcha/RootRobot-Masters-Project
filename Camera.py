# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
camera.start_preview() 
print('preview starting')
time.sleep(5) 
camera.capture('/home/pi/Desktop/shaiv.jpg')
camera.stop_preview()
# grab an image from the camera
#camera.capture(rawCapture, format="bgr")
#image = rawCapture.array
# display the image on screen and wait for a keypress
#cv2.imwrite(r'/home/pi/Desktop/sample.jpg', image)
#cv2.imshow("Image", image)
#cv2.waitKey(0)


