#!/usr/bin/env python3
"""Simple test code to publish to the Root a command"""

import rospy
from std_msgs.msg import String

def root_callback(msg):
    rospy.loginfo(' I heard %s', msg.data)

def main():
    """Node setup and main ROS loop"""
    rospy.init_node('RootTest', anonymous='True')

    #Prepare publisher on the 'sendRoot' topic
    pub = rospy.Publisher('toRoot', String, queue_size=10)
    rospy.Subscriber('fromRoot', String, root_callback)   # Use the 'sendRoot' topic
    msg = String()
    rate = rospy.Rate(1)  #update rate in Hz
    command=['red','green','blue']

    while not rospy.is_shutdown():
        for cmd in command:
            msg.data = cmd
            rospy.loginfo('%s(%.2f) %s' % (rospy.get_name(),rospy.get_time(),cmd))
            pub.publish(msg)
            rate.sleep()
            # msg.data = 'Stop'
            # rospy.loginfo('Stopping')
            # pub.publish('red')
            # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
