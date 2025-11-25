#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class ReconhecimentoCoresNode:
    def __init__(self):
        rospy.init_node("reconhecimento_dinheiro", anonymous=True)
        self.bridge = CvBridge()
        self.pub_dinheiro = rospy.Publisher("dinheiro", String, queue_size=1)
        self.sub_imagem = rospy.Subscriber("frame", Image, self.callback_imagem)
        rospy.loginfo("Nó de Reconhecimento de Cores iniciado. Aguardando imagens.")

    def callback_imagem(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Erro ao converter imagem: %s", e)
            return

        self.detectar_dinheiro(frame)

    def detectar_dinheiro(self, frame):
        frame = cv2.GaussianBlur(frame, (7,7), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cor_verde_baixo = np.array([35, 40, 40])
        cor_verde_alto  = np.array([90, 255, 255])
        mascara = cv2.inRange(hsv, cor_verde_baixo, cor_verde_alto)

        kernel = np.ones((7,7), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)

        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dinheiro_detectado = False

        for contorno in contornos:
            area = cv2.contourArea(contorno)
            if area < 8000:
                continue

            rect = cv2.minAreaRect(contorno)
            (w, h) = rect[1]
            if w == 0 or h == 0:
                continue

            proporcao = max(w, h) / min(w, h)
            if 1.2 < proporcao < 2.5:
                dinheiro_detectado = True
                break

        if dinheiro_detectado:
            self.pub_dinheiro.publish("DINHEIRO")
            rospy.loginfo("Dinheiro detectado!")
        else:
            self.pub_dinheiro.publish("nada")
            rospy.loginfo("nada...")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ReconhecimentoCoresNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
