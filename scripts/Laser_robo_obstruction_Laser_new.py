#!/usr/bin/env python

#
import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

def callback(msg):   #function callback which recieves a parameters named msg
  l = msg.ranges 
   
  list_l = list(l)  # List of the original ranges data
  r = rospy.Rate(10)

            # define lidar msg to new scan object


      # No change in the original Lidar data
  # If IR data is less than 0.4 
  for i in range(0,1079):
    if list_l[i]< 0.40:
      list_l[i] = 30
    else:
      list_l[i] = list_l[i]
    
      

  scan = LaserScan()  # Laserscan object
  scan = msg
  current_time = rospy.Time.now()
  scan.header.stamp = current_time
  scan.ranges = list_l
  scan.intensities = []
  scan_pub.publish(scan)
    
    
 




if __name__=='__main__':
  rospy.init_node('laser2')
  laser = rospy.Subscriber('/laser/scan', LaserScan, callback) # laser is a subscriiber object which listens to /mybot/laser/scan topic and it will call the function callback each time
  scan_pub = rospy.Publisher('/scan1', LaserScan, queue_size=1)
  rospy.spin()
  
  
  
 
  
  




