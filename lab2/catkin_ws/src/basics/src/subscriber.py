#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(msg):
    print(msg)


rospy.init_node("subscriber")
subscriber = rospy.Subscriber("hello_world", String, callback)
rospy.spin()
