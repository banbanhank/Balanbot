from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import serial

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
ser=serial.Serial("/dev/usb_arduino", 57600)
# allow the camera to warmup
time.sleep(0.1)
area_thresh = 170*170

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]

        shape = 0
        color = 0
        for cnt in cnts:
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            (x,y,w,h) = cv2.boundingRect(approx)
	    if(cv2.isContourConvex(approx) and w*h > area_thresh):
		if len(approx) == 3:
                    shape = 3
                elif len(approx) == 4 or len(approx)==5:
                    ar = w/float(h)
                    if(ar>=0.9 and ar<=1.1):
                        shape = 4
                elif len(approx) >= 7:
                    shape = 5
	cmd = str(shape)+','+str(color)+'\n'
        print(cmd)
	ser.write(cmd)
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

