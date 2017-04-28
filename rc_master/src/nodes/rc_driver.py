#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import RPi.GPIO as GPIO
import time

# set left motor pin
Motor1A = 13
Motor1B = 15
Motor1E = 18

# set right motor pin
Motor2A = 11
Motor2B = 12
Motor2E = 16



GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)

motorL = GPIO.PWM(Motor1E,100)
motorR = GPIO.PWM(Motor2E,100)
motorL.start(0)
motorR.start(0)

class ctrl_handler:

  def __init__(self):
    self.speedL = 0
    self.speedR = 0
    self.ctrl_sub = rospy.Subscriber('rc_ctrl', Twist, self.ctrl_callback)
    
  def ctrl_callback(self, data):
    self.speedL = data.linear.x
    self.speedR = data.linear.y
    self.set_left()
    self.set_right()
    #print data
    
  def set_left(self):
    #print "set_left"
    #print self.speedL
    if self.speedL > 0:
      #print "f"
      GPIO.output(Motor1A, GPIO.LOW)
      GPIO.output(Motor1B, GPIO.HIGH)
    if self.speedL < 0:
      #print "b"
      GPIO.output(Motor1A, GPIO.HIGH)
      GPIO.output(Motor1B, GPIO.LOW)
    if self.speedL == 0:
      motorL.ChangeDutyCycle(0)
    motorL.ChangeDutyCycle(abs(self.speedL))
    
  def set_right(self):
    #print "set_right"
    #print self.speedR
    if self.speedR > 0:
      #print "f"
      GPIO.output(Motor2A, GPIO.HIGH)
      GPIO.output(Motor2B, GPIO.LOW)
    if self.speedR < 0:
      #print "b"
      GPIO.output(Motor2A, GPIO.LOW)
      GPIO.output(Motor2B, GPIO.HIGH)
    if self.speedR == 0:
      motorR.ChangeDutyCycle(0)
    motorR.ChangeDutyCycle(abs(self.speedR))
    

    
if __name__ == "__main__":
    rospy.init_node('rc_driver', anonymous=True)
    ctrl_handler()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        motorR.stop()
        motorL.stop()
        GPIO.cleanup()
        print "Shutting Down"

