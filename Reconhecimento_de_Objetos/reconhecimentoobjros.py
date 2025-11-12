#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading


class ReconhecimentoCoresNode:
    def __init__(self):

        rospy.init_node('reconhecimento_cores_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        self.pub_fala = rospy.Publisher('/fala_cores', String, queue_size=10)
        
        self.pub_imagem_processada = rospy.Publisher('/reconhecimento/imagem_cores', Image, queue_size=10)
       

        self.sub_imagem = rospy.Subscriber('/camera/image_raw', Image, self.callback_imagem)
        

        self.ultima_fala = ""
        self.frequencia_fala = rospy.get_param('~frequencia_fala', 5) 
        self.tempo_ultima_fala = rospy.Time.now().to_sec()
        
        rospy.loginfo("Nó de Reconhecimento de Cores iniciado. Aguardando imagens.")

    def falar_se_necessario(self, texto):
        tempo_atual = rospy.Time.now().to_sec()
        

        if texto != self.ultima_fala or (tempo_atual - self.tempo_ultima_fala) > self.frequencia_fala:
            self.ultima_fala = texto
            self.tempo_ultima_fala = tempo_atual
            
            self.pub_fala.publish(String(texto))
            rospy.loginfo("Publicando fala: %s", texto)

    def processar_frame_cores(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        fala_para_dizer = None

        cor_verde_baixo = np.array([40, 50, 50])
        cor_verde_alto = np.array([80, 255, 255])
        mascara_verde = cv2.inRange(hsv, cor_verde_baixo, cor_verde_alto)
        contornos_verdes, _ = cv2.findContours(mascara_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contorno in contornos_verdes:
            perimetro = cv2.arcLength(contorno, True)
            aproximacao = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
            
            if len(aproximacao) == 4:
                x, y, w, h = cv2.boundingRect(aproximacao)
                if w > 50 and h > 50:
                    proporcao = float(w) / h
                    if proporcao > 1.2 and proporcao < 1.8:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(frame, "Plankton (Verde)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        fala_para_dizer = "Plankton"
                        break

        if fala_para_dizer is None:
            cor_amarelo_baixo = np.array([20, 100, 100])
            cor_amarelo_alto = np.array([30, 255, 255])
            mascara_amarelo = cv2.inRange(hsv, cor_amarelo_baixo, cor_amarelo_alto)
            contornos_amarelos, _ = cv2.findContours(mascara_amarelo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contorno in contornos_amarelos:
                x, y, w, h = cv2.boundingRect(contorno)
                if w > 50 and h > 50:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, "Bob Esponja (Amarelo)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    fala_para_dizer = "Bob Esponja"
                    break

        if fala_para_dizer is None:
            cor_vermelho_baixo1 = np.array([0, 100, 100])
            cor_vermelho_alto1 = np.array([10, 255, 255])
            cor_vermelho_baixo2 = np.array([170, 100, 100])
            cor_vermelho_alto2 = np.array([180, 255, 255])
            
            mascara_vermelho1 = cv2.inRange(hsv, cor_vermelho_baixo1, cor_vermelho_alto1)
            mascara_vermelho2 = cv2.inRange(hsv, cor_vermelho_baixo2, cor_vermelho_alto2)
            mascara_vermelho = cv2.bitwise_or(mascara_vermelho1, mascara_vermelho2)
            
            contornos_vermelhos, _ = cv2.findContours(mascara_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contorno in contornos_vermelhos:
                x, y, w, h = cv2.boundingRect(contorno)
                if w > 50 and h > 50:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, "Seu Siriguejo (Vermelho)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    fala_para_dizer = "Seu Siriguejo"
                    break

        return frame, fala_para_dizer

    def callback_imagem(self, data):
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        frame_processado, fala = self.processar_frame_cores(cv_image)
        

        if fala:
            self.falar_se_necessario(fala)

    
        try:
            self.pub_imagem_processada.publish(self.bridge.cv2_to_imgmsg(frame_processado, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
            
    
        cv2.imshow("Reconhecimento de Cores", frame_processado)
        cv2.waitKey(3) 

    def run(self):

        rospy.spin()

if __name__ == '__main__':
    try:
        node = ReconhecimentoCoresNode()
        node.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass