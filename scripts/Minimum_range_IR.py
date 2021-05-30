#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Range


a = 0
def callback0(msg): # call Back for IR data
  global a
  a = msg.range
b = 0
def callback1(msg): # call Back for IR data
  global b
  b = msg.range

c = 0
def callback2(msg): # call Back for IR data
  global c
  c = msg.range
d = 0
def callback3(msg): # call Back for IR data
  global d
  d = msg.range
e = 0
def callback4(msg): # call Back for IR data
  global e
  e = msg.range
f = 0
def callback5(msg): # call Back for IR data
  global f
  f = msg.range

g = 0
def callback6(msg): # call Back for IR data
  global g
  g = msg.range

h = 0
def callback7(msg): # call Back for IR data
  global h
  h = msg.range
  m = min(a,b,c,d,e,f,g,h)
  print m


if __name__=='__main__':
  rospy.init_node('laser1')
  laser = rospy.Subscriber('/range_0',Range, callback0) 
  ir0 = rospy.Subscriber('/range_1', Range, callback1)
  ir1 = rospy.Subscriber('/range_2', Range, callback2)
  ir2 = rospy.Subscriber('/range_3', Range, callback3)
  ir3 = rospy.Subscriber('/range_4', Range, callback4)
  ir4 = rospy.Subscriber('/range_5', Range, callback5)
  ir5 = rospy.Subscriber('/range_6', Range, callback6)
  ir6 = rospy.Subscriber('/range_7', Range, callback7)

  rospy.spin()
  
  
  
 
  
  




