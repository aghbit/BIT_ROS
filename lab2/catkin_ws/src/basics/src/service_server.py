#!/usr/bin/env python3
import rospy
from basics.srv import WordCount, WordCountResponse


def count_words(request):
    return WordCountResponse(len(request.words.split()))


rospy.init_node('service_server')
service = rospy.Service('word_count', WordCount, count_words)
rospy.spin()
