#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # pub = rospy.Publisher('status', Twist, queue_size=1)
    # msg = Twist()
    x = data.data
    if(x==1):
        print("emergency stop")
    if(x==2):
        print("light off")
    if(x==0):
        print("continue")

    
def status():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("status", Float32, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    status()