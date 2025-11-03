#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cap = cv2.VideoCapture(0)

def publish_camera(cap):
    pub = rospy.Publisher("frame",Image,queue_size=10)
    rospy.init_node("camera_pub")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("Erro na abertura da câmera.")
            break
        else: 
            try:
                msg = bridge.cv2_to_imgmsg(frame, "bgr8")           #Conversão da imagem cv2 para msg
            except CvBridgeError:
                rospy.loginfo("Erro na conversão de imagem.")
            else:
                pub.publish(msg)
                rospy.loginfo("Câmera Publicada!")
    rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera(cap)
        cap.release()
    except rospy.ROSInterruptException:
        pass
