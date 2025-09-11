#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

def callback(data):
    rospy.loginfo("Imagem recebida!")
    frame = bridge.imgmsg_to_cv2(data.data,"bgr8")
    cv2.imshow("frame", frame)

def show_image():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber('camera_pub', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    show_image()