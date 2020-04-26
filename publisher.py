#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import json


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    n = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % str(n)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        n = n+1
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass