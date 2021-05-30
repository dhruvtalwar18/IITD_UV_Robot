from __future__ import division

import cv2

import rospy
import argparse
import sys
import time
import math
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
#from statistics import mean 


parser = argparse.ArgumentParser()
parser.add_argument("--path", type=str, help="Video path")

def main():
    pub = rospy.Publisher('/zed/image/compressed', CompressedImage, queue_size=10)
    bridge = CvBridge()
    rospy.init_node('camera', anonymous=True)
    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()   
    ticks = 0

    #lt = track.LaneTracker(2, 0.1, 500)
    #ld = detect.LaneDetector(180)
    #ld2 = detect2.LaneDetector(180)
    height, width = frame.shape[:2]
    #fps = int(8)    
    #fourcc = cv2.VideoWriter_fourcc(*'MPEG')
    #out = cv2.VideoWriter(name, fourcc, fps, (width,height))
    p=[(0,0,0,0),(0,0,0,0)]
    l2_old=0
    x_old=0
    m1=0
    m2=0
    m=0
    arr1=[]
    arr2=[]
    value=0
    value_old=0
    while cap.isOpened():
    	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    	ros_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    	msg = CompressedImage()
    	msg.header.stamp = rospy.Time.now()
    	msg.format = "jpeg"
    	msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
    	#rospy.loginfo(ros_msg)
        pub.publish(msg
            )
        precTick = ticks
        ticks = cv2.getTickCount()
        dt = (ticks - precTick) / cv2.getTickFrequency()
        #print(dt)
        ret, frame = cap.read()
        frame = frame[100:600,300:1100]
        #predicted = lt.predict(dt)
        #time.sleep(0.02)
        f=frame[0:400]
        cv2.imshow('', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    #args = parser.parse_args()
    main()
