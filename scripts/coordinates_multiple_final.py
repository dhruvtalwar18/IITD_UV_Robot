#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def callbackx(datax):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", datax.data)
    funx(datax)

def callbacky(datay):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", datay.data)
    funy(datay)

def callback(data):
    if(data.data==1):
        autonomous_movement()

def autonomous_movement():
    global pub
    global pub1
    global j

    pub = rospy.Publisher('robot_status', String, queue_size=10)
    pub1 = rospy.Publisher('work_comp', Float32, queue_size=10)
    time.sleep(2)
    for i in range(0,len(arrx)):
        j = round((i*100.0/len(arrx)),2)
        print(j)
        pub1.publish(j)
        pub.publish("Moving")
        a=[((1.6-((arrx[i].data)/(80.0))),(((arry[i].data)*7/(800.0))-2.8),0.0),(0.0,0.0,0.0,1.0)]
        goal = goal_pose(a)
        client.send_goal(goal)
        client.wait_for_result()
        pub.publish("Sanitation")
        print('ss')
        time.sleep(5)
    pub1.pu         
    pub.publish("Waiting")


def goal_pose(pose):  # <2>
    print(pose)
    pub.publish("Moving")
    pub1.publish(j)
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    print('hi')
    return goal_pose



def funx(data):

    global arrx
    arrx.append(data)
    print(arrx)


def funy(data):

    global arry
    arry.append(data)
    print(arry)
    
def coordinates():
    print("h")
    rospy.Subscriber("x_nodes", Float32, callbackx)
    rospy.Subscriber("y_nodes", Float32, callbacky)
    rospy.Subscriber("autonomous", Float32, callback)
    
    
    rospy.spin()

if __name__ == '__main__': 
    rospy.init_node('listener', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()   
    arrx = []
    arry = []
    coordinates()
