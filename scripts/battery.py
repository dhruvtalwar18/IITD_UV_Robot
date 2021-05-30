#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('battery', Int32, queue_size=10)
    rospy.init_node('battery', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(80)
        pub.publish(80)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
