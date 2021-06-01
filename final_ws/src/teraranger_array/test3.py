#!/usr/bin/env python

# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
import tf2_ros

from sensor_msgs.msg import Range
from std_msgs.msg import Header
from teraranger_array.msg import RangeArray
import message_filters



def getmin(msg):
	outlist = []
	for i in range(8):
		outlist.append(max(msg.ranges[i].range, 0))
	minval = min(outlist)
	print("minimum: ", minval)
	outval = Range()
	outval.header = msg.header
	outval.range = minval
	outval.radiation_type = 1
	outval.field_of_view = msg.ranges[0].field_of_view
	outval.min_range = msg.ranges[0].min_range
	outval.max_range = msg.ranges[0].max_range
	min_val_publish.publish(outval)
	return outval

if __name__ == '__main__':
	rospy.init_node('terabee_reader')
	min_val = rospy.Subscriber('ranges', RangeArray, getmin)
	min_val_publish = rospy.Publisher('min_terabee', Range, queue_size=1)
	rospy.spin()
