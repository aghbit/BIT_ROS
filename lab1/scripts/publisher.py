#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node("publisher")
publisher = rospy.Publisher("/hello_world", String, queue_size=10)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    publisher.publish("Hello World")
    r.sleep()
