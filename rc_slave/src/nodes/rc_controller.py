#!/usr/bin/env python

from Tkinter import *
import roslib
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys



class Playfield:

  def __init__(self):
    
    self.pressed = {}
    self._create_ui()
    self.twist = Twist()
    self.speed = 0
    self.maxSpeed = 25
    self.pub = rospy.Publisher('rc_keyboard', Twist, queue_size=1)
    self.twist.linear.x = 0
    self.twist.linear.y = 0
    
  def start(self):
    self._animate()
    self.root.mainloop()
    
  def _create_ui(self):
    self.root = Tk()
    self.canvas = Canvas(width=440, height=440)
    self.canvas.config(scrollregion=(-20,-20,420,420))
    self.canvas.pack(side="top", fill="both", expand="true")
    
    self.leftRect = Paddle(self.canvas, tag="l",
    color="blue", x=150, y=200)
    self.rightRect = Paddle(self.canvas, tag="r",
    color="blue", x=250, y=200)
    self.upRect = Paddle(self.canvas, tag="u",
    color="blue", x=200, y=150)
    self.downRect = Paddle(self.canvas, tag="d",
    color="blue", x=200, y=250)
    
    self._set_bindings()
    
  def _animate(self):
    if self.speed < self.maxSpeed:
      self.speed += 5
    if self.pressed["w"] and self.pressed["a"]:
      self.fwLeft()
    elif self.pressed["w"] and self.pressed["d"]:
      self.fwRight()
    elif self.pressed["a"] and self.pressed["s"]:
      self.bwLeft()
    elif self.pressed["s"] and self.pressed["d"]:
      self.bwRight()
    elif self.pressed["w"]:
      self.fw()
    elif self.pressed["s"]:
      self.bw()
    elif self.pressed["a"]:
      self.left()
    elif self.pressed["d"]:
      self.right()
    else:
      self.speed = 0
      self.idle()
    
    if not self.pressed["a"]:
      self.leftRect.redraw()
    if not self.pressed["d"]:
      self.rightRect.redraw()
    if not self.pressed["w"]:
      self.upRect.redraw()
    if not self.pressed["s"]:
      self.downRect.redraw()
    self.pub.publish(self.twist)
    self.root.after(30, self._animate)
    
      
  def fwLeft(self):
    #print "fwLeft"
    self.leftRect.redraw("red")
    self.upRect.redraw("red")
    self.twist.linear.x = int(self.speed * 0.5)
    self.twist.linear.y = self.speed
    self.twist.linear.z = self.maxSpeed
    
  def fw(self):
    #print "fw"
    self.upRect.redraw("red")
    self.twist.linear.x = self.speed
    self.twist.linear.y = self.speed
    self.twist.linear.z = self.maxSpeed
  
  def fwRight(self):
    #print "fwRight"
    self.rightRect.redraw("red")
    self.upRect.redraw("red")
    self.twist.linear.x = self.speed
    self.twist.linear.y = int(self.speed * 0.5)
    self.twist.linear.z = self.maxSpeed
  
  def bwLeft(self):
    #print "bwLeft"
    self.downRect.redraw("red")
    self.leftRect.redraw("red")
    self.twist.linear.x = int(-self.speed * 0.7)
    self.twist.linear.y = -self.speed
    self.twist.linear.z = self.maxSpeed
    
  def bwRight(self):
    #print "bwRight"
    self.downRect.redraw("red")
    self.rightRect.redraw("red")
    self.twist.linear.x = -self.speed
    self.twist.linear.y = int(-self.speed * 0.7)
    self.twist.linear.z = self.maxSpeed
    
  def bw(self):
    #print "bw"
    self.downRect.redraw("red")
    self.twist.linear.x = -self.speed
    self.twist.linear.y = -self.speed
    self.twist.linear.z = self.maxSpeed
    
  def left(self):
    self.leftRect.redraw("red")
    self.twist.linear.x = 0
    self.twist.linear.y = self.speed * 0.5
    self.twist.linear.z = self.maxSpeed
    
  def right(self):
    self.rightRect.redraw("red")
    self.twist.linear.x = self.speed * 0.5
    self.twist.linear.y = 0
    self.twist.linear.z = self.maxSpeed
    
  def idle(self):
    #print "idle"
    self.twist.linear.x = 0
    self.twist.linear.y = 0
    self.twist.linear.z = self.maxSpeed
    
    
  def _set_bindings(self):
    for char in ["w","a", "s", "d"]:
      self.root.bind("<KeyPress-%s>"%char, self._pressed)
      self.root.bind("<KeyRelease-%s>"%char, self._released)
      self.pressed[char] = False
      
  def _pressed(self, event):
    self.pressed[event.char] = True
  
  def _released(self, event):
    self.pressed[event.char] = False
    

class Paddle():
  def __init__(self, canvas, tag, color="blue", x=0, y=0):
    self.canvas = canvas
    self.tag = tag
    self.x = x
    self.y = y
    self.redraw()
  
  def redraw(self, color="blue"):
    x0 = self.x - 20
    x1 = self.x + 20
    y0 = self.y - 10
    y1 = self.y + 10
    self.canvas.delete(self.tag)
    self.canvas.create_rectangle(
    x0,y0,x1,y1,tags=self.tag, fill=color)
  
if __name__== "__main__":
    rospy.init_node('rc_key', anonymous=True)
    p = Playfield()
    p.start()
