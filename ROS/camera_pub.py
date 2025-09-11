#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cap = cv2.VideoCapture(0)

def camera_pub(cap):
    pub = rospy.Publisher("camera_pub",Image,queue_size=10)
    rospy.init_node("camera_pub", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        rospy.loginfo("Câmera Publicada!")
        if not ret:
            rospy.loginfo("Erro na abertura da câmera.")
            break
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)
        if cv2.waitKey(1) == ord('q'):
            break
    rate.sleep()

if __name__ == '__main__':
    try:
        camera_pub(cap)
        cap.release()
    except rospy.ROSInterruptException:
        pass
