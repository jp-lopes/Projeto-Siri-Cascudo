#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def asking():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        question = input('How old are you? ')
        rospy.loginfo(question)
        pub.publish(question)
        rate.sleep()

if __name__ == '__main__':
    try:
        asking()
    except rospy.ROSInterruptException:
        pass