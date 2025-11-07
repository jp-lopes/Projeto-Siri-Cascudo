#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cap = None
parar_flag = False

def callback(data):
    global parar_flag
    if data.data == 'CAMERA':
        parar_flag = True
        rospy.loginfo("Captura ativada.")
    elif data.data == 'DESATIVAR':
        parar_flag = False
        rospy.loginfo("Captura desativada.")

def cam_pub():
    global cap
    rospy.init_node("camera_pub")
    pub = rospy.Publisher("frame", Image, queue_size=10)
    rospy.Subscriber("comandos", String, callback)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            if parar_flag:
                if cap is None or not cap.isOpened():
                    cap = cv2.VideoCapture(0)

                ret, frame = cap.read()
                if not ret:
                    rospy.logwarn("Erro ao capturar frame.")
                else:
                    try:
                        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                        pub.publish(msg)
                        rospy.loginfo("Frame publicado!")
                    except CvBridgeError as e:
                        rospy.logerr(f"Erro na conversão: {e}")
            else:
                if cap is not None and cap.isOpened():
                    cap.release()

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Encerrando nó...")
        if cap is not None and cap.isOpened():
            cap.release()

if __name__ == '__main__':
    try:
        cam_pub()
    except rospy.ROSInterruptException:
        pass
