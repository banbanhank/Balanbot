#! /usr/bin/env python
import argparse
import imutils
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from balanbot.msg import Detect
import numpy as np

class Detector(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber("/camera_node/image/compressed",CompressedImage,self.cb_img,queue_size=1)
        #self.pub_img = rospy.Publisher("/detector/image/compressed",CompressedImage,queue_size=1)
        self.area_thresh = 9000
        self.font = cv2.FONT_HERSHEY_COMPLEX

        self.pub_dt = rospy.Publisher("/detection",Detect,queue_size=1)

    def cb_img(self,msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]
        
        shape = 0
        color = 0
        for cnt in cnts:
	    approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            if(cv2.isContourConvex(approx) and cv2.contourArea(approx)>self.area_thresh):
                #mask = np.zeros(image.shape[:2], np.uint8)
                #cv2.drawContours(image, [approx], 0, 0, 3)
                #mean_val = cv2.mean(image, mask)
                #color = self.color_label(mean_val)
                #print(mean_val)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                if len(approx) == 3:
                    #cv2.drawContours(image, [approx], 0, 0, 3)
                    shape = 3
                elif len(approx) == 4:
                    (x,y,w,h) = cv2.boundingRect(approx)
                    ar = w/float(h)
                    if(ar>=0.9 and ar<=1.1):
                       # cv2.drawContours(image, [approx], 0, 0, 3)
                        shape = 4
                else:
                    shape = 5
        
        dt = Detect()
        dt.shape = shape
        dt.color = color
        self.pub_dt.publish(dt) 
        #self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(image))
        
    def color_label(self,mean):
        #print(mean)
        if(mean[2]>mean[1] and mean[2]>mean[0]):
            return 1
        elif(mean[1]>mean[2] and mean[1]>mean[0] and (mean[1]-mean[0])>20):
            return 2
        else:
            return 3

    def on_shutdown(self):
        pass
    
if __name__ == "__main__":
    rospy.init_node("detector")
    detector = Detector()
    rospy.on_shutdown(detector.on_shutdown)
    rospy.spin()
    
