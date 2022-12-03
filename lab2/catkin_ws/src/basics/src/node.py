#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("node")
    rospy.loginfo("Hello World!")
    rospy.spin()

if __name__ == "__main__":
    main()