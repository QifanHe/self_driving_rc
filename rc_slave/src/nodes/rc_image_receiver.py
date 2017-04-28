#!/usr/bin/env python
import numpy as np
from operator import itemgetter
import sys
import cv2
import roslib
from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import CompressedImage
from medianFilter import median_2d_time_filter




class object_detector:
  def __init__(self, path = "object_classifier/stopsignsDetector.xml", scaleFactor=1.3, minNeighbors=3, minSize = (40,40)):
    self.cascade = cv2.CascadeClassifier('/home/qifan/selfDrivingRC/src/nodes/stopsignsDetector.xml')
    self.median = median_2d_time_filter()
    self.scaleFactor = scaleFactor
    self.minNeighbors = minNeighbors
    self.minSize = minSize

    
  def detect(self, image):
    objects=self.cascade.detectMultiScale(
        image,
        scaleFactor=self.scaleFactor,
        minNeighbors=self.minNeighbors,
        minSize=self.minSize,
        flags = cv2.CASCADE_SCALE_IMAGE
    )
    
    if type(objects) == tuple:
      self.median.push(0,0,0,0)
    else:
      ojbects = sorted(objects.tolist(), key=itemgetter(3))
      self.median.push(objects[-1][0], objects[-1][1], objects[-1][2], objects[-1][3])
    return self.median.median()



class image_handler:

  def __init__(self):
    self.sub = rospy.Subscriber("/image_bridge", CompressedImage, self.callback, queue_size=1)
    self.pubLane = rospy.Publisher("lanePos", Twist, queue_size=1)
    self.pubStop = rospy.Publisher("stopPos", Twist, queue_size=1)
    self.stopSign = object_detector("stopsignsDetector.xml", 1.1, 3, (45,45))
    self.switcher = 0
    self.stop = Twist()
    self.lane = Twist()
    self.direction = 0
    
    
    
    
    
  def callback(self, ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if self.switcher == 0:
      img = self.detect_draw(img)
      self.switcher = 1
      #self.detect_lane(img)
      #cv2.imwrite('/home/qifan/frame.png', img)
      cv2.imshow('frame', img)
      cv2.waitKey(1)
    else:
      self.switcher = 0
      
    
    
  def detect_draw(self, image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    stops = self.stopSign.detect(gray)
    x, y, w, h = stops
    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    self.stop.linear.x = x
    self.stop.linear.y = y
    self.stop.linear.z = h
    self.pubStop.publish(self.stop)
    return image
    
  def detect_lane(self, image):
    samples = [265,267,269,271,273]
    results = []
    blur = cv2.GaussianBlur(image,(5,5),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([0, 50, 50])
    upper_yellow = np.array([30, 255, 255])
    
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    kernel = np.ones((2,2),np.uint8)
    #mask = cv2.erode(mask,kernel,iterations = 1)
    #mask = cv2.dilate(mask,kernel,iterations = 1)
    
    
    for y in samples:
      cv2.line(image, (0, y), (639, y), (0,0,0))
      tmp = 0
      pixel = 0
      for x in range(0, 640):
        if mask[y, x]:
          pixel += 1
        else:
          if pixel:
            pixel -= 1
        if pixel == 2:
          tmp = x
          results.append((tmp, y))
          cv2.rectangle(image, (tmp-1, y-1), (tmp+1, y+1), (0, 255, 0), 2)
          break
    tmpx = 0
    tmpy = 0
    if len(results) > 2:
      for i in range(3):
        tmpx += results[i][0]
        tmpy += results[i][1]
      self.direction = tmpx/3
    else:
      self.direction = 0
    cv2.rectangle(image, (self.direction, tmpy/3 - 1), (320, tmpy/3 + 1), (0, 255, 255), 2)
    self.lane.linear.x = self.direction
    self.pubLane.publish(self.lane)
    #print results
    #print mask.shape
    
    
    
    #cv2.imshow('mask', mask)
    cv2.imshow('blur', blur)
    #cv2.imshow('image', image)
    #cv2.waitKey(1)
    
def main(args):
    ih = image_handler()
    rospy.init_node('image_handler', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
