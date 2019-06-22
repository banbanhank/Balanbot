#! /usr/bin/env python

import rospy
import serial
from balanbot.msg import Motor_cmd

class Driver:
    def __init__(self):
        self.ser=serial.Serial("/dev/usb_arduino", 57600)
        self.sub = rospy.Subscriber('/motor_cmd',Motor_cmd,self.cb_cmd,queue_size=1)
        
    def cb_cmd(self,msg):
        self.ser.write(str(msg.lj)+','+str(msg.rj)+'\n')

    def on_shutdown(self):
        self.ser.close()


if __name__ == "__main__":
    rospy.init_node('driver')
    driver = Driver()
    rospy.on_shutdown(driver.on_shutdown)
    rospy.spin()