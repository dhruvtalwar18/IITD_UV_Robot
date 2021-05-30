#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub = rospy.Publisher('/UVBot/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    x = data.data
    if(x==1):
        msg.linear.x=-0.5
        msg.angular.z=0
        pub.publish(msg)
    if(x==2):
        msg.angular.z=-0.5
        msg.linear.x=0
        pub.publish(msg)
    if(x==3):
        msg.angular.z=0.5
        msg.linear.x=0
        pub.publish(msg)
    if(x==4):
        msg.linear.x=0.5
        msg.angular.z=0
        pub.publish(msg)
    if(x==0):
        msg.linear.x=0
        msg.angular.z=0
        pub.publish(msg)
    # time.sleep(0.1)

    
def throttle():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("throttle", Float32, callback)
    
    rospy.spin()

if __name__ == '__main__':
    throttle()
