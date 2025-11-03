#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class CameraNode:
    def __init__(self):
        self.ativo = False
        self.cap = None
        self.bridge = CvBridge()


    def callback(self, data):
        comando = data.data
        
        if comando == 'ATIVAR CAMERA':
            if not self.ativo:
                self.ativo = True
                rospy.loginfo("CAMERA ATIVADA!")

        elif comando == 'DESATIVAR CAMERA':
            if self.ativo:
                self.ativo = False
                rospy.loginfo("CAMERA DESATIVADA!")


    def publish_camera(self):
        rospy.init_node("camera_pub")
        pub = rospy.Publisher("frame",Image,queue_size=10)
        rate = rospy.Rate(10)
        rospy.Subscriber('comando_detectado', String, self.callback)
        
        try:
            while not rospy.is_shutdown():
                if self.ativo:
                    if self.cap is None or not self.cap.isOpened():
                        self.cap = cv2.VideoCapture(0)
                    ret, frame = self.cap.read()
                    if not ret:
                        rospy.loginfo("Erro na abertura da câmera.")
                    else:
                        try:
                            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")  #Conversão da imagem cv2 para msg
                        except CvBridgeError:
                            rospy.loginfo("Erro na conversão de imagem.")
                        else:
                            pub.publish(msg)
                            rospy.loginfo("Câmera Publicada!")
                else:
                    if self.cap is not None and self.cap.isOpened():
                        self.cap.release()
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            rospy.loginfo("Encerrando nó...")
            if self.cap.isOpened():
                self.cap.release()


if __name__ == '__main__':
    try:
        camera_pub = CameraNode()
        camera_pub.publish_camera()
    except rospy.ROSInterruptException:
        pass