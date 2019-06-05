#! /usr/bin/env python
import argparse
import imutils
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from shape_detector import ShapeDetector

class Detector(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.bridge = CvBridge()
        self.sd = ShapeDetector()
        self.sub_img = rospy.Subscriber("/camera_node/image/compressed",CompressedImage,self.cb_img,queue_size=1)
        self.pub_img = rospy.Publisher("/detector/image/compressed",CompressedImage,queue_size=1)

    def cb_img(self,msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        resized = imutils.resize(image, width=300)
        ratio = image.shape[0] / float(resized.shape[0])
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect(c)
            
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 2)

            #cv2.imshow("Image", image)
            #cv2.waitKey(0)
        
        self.pub_img.publish(self.bridge.cv2_to_compressed_imgmsg(image))
        

    def on_shutdown(self):
        pass
    
if __name__ == "__main__":
    rospy.init_node("detector")
    detector = Detector()
    rospy.on_shutdown(detector.on_shutdown)
    rospy.spin()
    