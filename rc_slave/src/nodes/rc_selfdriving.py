#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from collections import deque

class selfDriving:
  
  def __init__(self):
    self.autocontrol = True
    self.lineKeeping = True
    self.stopControl = False
    self.lineKeepingAvailable = False
    self.ctrl_pub = rospy.Publisher('rc_ctrl', Twist,
    queue_size=1)
    self.keyboard_sub = rospy.Subscriber('rc_keyboard', Twist, self.keyboardMsgCallback, queue_size=1)
    self.lane_sub = rospy.Subscriber('lanePos', Twist, self.laneMsgCallback, queue_size=1)
    self.stop_sub = rospy.Subscriber('stopPos', Twist, self.stopMsgCallback, queue_size=1)
    self.ctrlData = Twist()
    self.twist = Twist()
    self.steer = 0
    self.stopMiss = 0
    self.maxSpeed = 0
    self.ctrlSpeed = [0, 0]
    self.timeCounter = 0
    self.stopped = 0
    self.stop = {"x": 0, "y":0, "z":0}
    self.stopQueue = deque([0,0,0,0,0,0,0,0,0,0])
    
  
  def keyboardMsgCallback(self, data):
    self.maxSpeed = data.linear.z
    self.ctrlSpeed = [data.linear.x, data.linear.y]
    if self.autocontrol:
      self.autoStop()
    else:
      self.forward(data)
    #elif self.autocontrol:
    #  if self.lineKeeping == True:
    #    self.laneKeeping()
      
  
  def stopMsgCallback(self, data):
    self.stop['x'] = data.linear.x
    self.stop['y'] = data.linear.y
    self.stop['z'] = data.linear.z
    self.stopQueue.append(data.linear.z)
    self.stopQueue.popleft()
  
  def laneMsgCallback(self, data):
    direction = data.linear.x
    if direction:
      self.steer = direction - 320
    else:
      self.steer = 0
    
  def laneKeeping(self):
    if self.timeCounter > 30:
        if self.steer > 150:
          propotional_term = 0.4
        elif self.steer < -150:
          propotional_term = -0.4
        else:
          propotional_term = 0.4 * (self.steer / 150)
        if propotional_term >= 0:
          self.twist.linear.x = self.ctrlSpeed[0]
          self.twist.linear.y = self.ctrlSpeed[1]*(1-propotional_term)
        else:
          self.twist.linear.x = self.ctrlSpeed[0]*(1+propotional_term)
          self.twist.linear.y = self.ctrlSpeed[1]
    else:
        self.twist.linear.x = self.ctrlSpeed[0]
        self.twist.linear.y = self.ctrlSpeed[1]
    self.forward(self.twist)
    
  def autoStop(self):
    if self.stop['z'] > 70 and (self.ctrlSpeed[0]>0 or self.ctrlSpeed[1]>0):
      self.stopped = 1
      print "stop"
    elif self.stopMiss:
      if self.timeCounter > 0:
        self.timeCounter -= 1
      else:
        self.stopped = 1
        self.stopMiss = 0
      print "missing"
      self.twist.linear.x = 0
      self.twist.linear.y = 0
    else:
      self.twist.linear.x = self.ctrlSpeed[0]
      self.twist.linear.y = self.ctrlSpeed[1]
      index1 = [i for i, e in enumerate(self.stopQueue) if e != 0]
      index2 = [i for i, e in enumerate(self.stopQueue) if e == 0]
      if sum(index1) == 10 and sum(index2) == 35:
        self.stopMiss = 1
        print "stopMiss"
        self.timeCounter = 50
    if self.stopped:
      self.twist.linear.x = 0
      self.twist.linear.y = 0
    if self.ctrlSpeed[0] <= 0 or self.ctrlSpeed[1] <= 0:
      self.stopped = 0
      self.stopQueue = deque([0,0,0,0,0,0,0,0,0,0])
    self.forward(self.twist)
  
    
  def forward(self, data):
    self.ctrl_pub.publish(data)
    #print data

  
if __name__ == "__main__":
    rospy.init_node('rc_selfdriving', anonymous=True)
    selfDriving()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
  
  



