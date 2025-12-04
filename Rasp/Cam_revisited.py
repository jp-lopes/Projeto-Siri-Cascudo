#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cap = None

def abrir_camera():
    cap = cv2.VideoCapture(-1, cv2.CAP_V4L2)
    if not cap.isOpened():
        rospy.logerr("Falha ao abrir a câmera.")
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    rospy.loginfo("Câmera ligada.")
    return cap

def cam_pub():
    global cap
    rospy.init_node("camera_pub2")
    pub = rospy.Publisher("frame", Image, queue_size=1)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():

            if cap is None:
                cap = abrir_camera()
                rate.sleep()
                continue

            cap.grab()
            ret, frame = cap.retrieve()

            if not ret:
                rospy.logwarn("Erro ao capturar frame. Reiniciando câmera...")
                cap.release()
                cap = None
                rospy.sleep(0.3)
                continue

            try:
                msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(msg)
            except CvBridgeError as e:
                rospy.logerr(f"Erro conversão: {e}")

            rate.sleep()

    finally:
        rospy.loginfo("Encerrando nó...")
        if cap is not None:
            cap.release()

if __name__ == '__main__':
    cam_pub()
