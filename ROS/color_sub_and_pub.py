#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from algelin_colors import coor

bridge = CvBridge()

def callback(imgmsg, pub):
    try:
        frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")  # Conversao da imagem msg para cv2
    except CvBridgeError:
        rospy.loginfo("Erro na conversao da imagem.")
        return

    height, width, _ = frame.shape
    point = frame[int(height/2), int(width/2)]
    point2 = np.array(point).tolist()

    color = coor(point2)
    resposta = color.escolha()

    if resposta is not None:
        rospy.loginfo("Cor detectada: " + resposta)
        pub.publish(resposta)  # publica a cor detectada
    else:
        rospy.loginfo("Desconhecida")
        pub.publish("Desconhecida")

def detect_color():
    rospy.init_node('detect_color')

    pub = rospy.Publisher('cor_detectada', String, queue_size=10)

    # passa o publisher como argumento extra para o callback
    rospy.Subscriber('camera_pub', Image, callback, callback_args=pub)

    rospy.spin()

if __name__ == '__main__':
    detect_color()
