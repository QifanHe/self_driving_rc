#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

#bridge = CvBridge()
cap = cv2.VideoCapture(0)

def getImage():
    ret, frame = cap.read()
    frame=cv2.flip(frame,0)
    frame=cv2.flip(frame,1)
    return frame

def sender():
    pub = rospy.Publisher('image_bridge', CompressedImage, queue_size=1)
    rospy.init_node('image_sender', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        img = getImage()
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        #print img.shape
        pub.publish(msg)
        rate.sleep()
      
if __name__=='__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
    cap.release()        
  
