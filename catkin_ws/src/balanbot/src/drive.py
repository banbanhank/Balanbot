#! /usr/bin/env python

import rospy
import serial
from balanbot.msg import Motor_cmd, Detect

class Driver:
    def __init__(self):
        self.ser=serial.Serial("/dev/usb_arduino", 57600)
        self.sub = rospy.Subscriber('/detection',Detect,self.cb_cmd,queue_size=1)
            

    def cb_cmd(self,msg):
        cmd = str(msg.shape)+','+str(msg.color)+'\n'
        self.ser.write(cmd)
        print(cmd)

    def on_shutdown(self):
        self.ser.close()


if __name__ == "__main__":
    rospy.init_node('driver')
    driver = Driver()
    rospy.on_shutdown(driver.on_shutdown)
    rospy.spin()
