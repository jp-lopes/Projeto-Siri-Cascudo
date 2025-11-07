#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

parar_flag = True

class color:
    def __init__(self, name, lower, upper):
        self.name = name
        self.lower = lower
        self.upper = upper

black   = color("Black",            np.array([0, 0, 0]),        np.array([180, 255, 50]))
gray    = color("Gray",             np.array([0, 0, 50]),       np.array([180, 50, 200]))
white   = color("White",            np.array([0, 0, 200]),      np.array([180, 30, 255]))
red1    = color("Red",              np.array([0, 50, 50]),      np.array([10, 255, 255]))
red2    = color("Red",              np.array([170, 50, 50]),    np.array([180, 255, 255]))
orange  = color("Orange",           np.array([10, 50, 50]),     np.array([20, 255, 255]))
beige   = color("Beige",            np.array([20, 30, 150]),    np.array([30, 100, 255]))
yellow  = color("Yellow",           np.array([25, 50, 50]),     np.array([35, 255, 255]))
green        = color("Green",       np.array([35, 50, 50]),     np.array([65, 255, 255]))
light_green  = color("LightGreen",  np.array([50, 30, 150]),    np.array([65, 150, 255]))
cyan       = color("Cyan",          np.array([65, 50, 50]),     np.array([85, 255, 255]))
light_blue = color("LightBlue",     np.array([85, 50, 150]),    np.array([100, 150, 255]))
blue       = color("Blue",          np.array([100, 50, 50]),    np.array([130, 255, 255]))
purple  = color("Purple",           np.array([130, 50, 50]),    np.array([150, 255, 255]))
magenta = color("Magenta",          np.array([150, 50, 50]),    np.array([160, 255, 255]))
pink    = color("Pink",             np.array([160, 50, 150]),   np.array([170, 150, 255]))
brown = color("Brown",              np.array([10, 50, 50]),     np.array([20, 255, 150]))

colors_list = [
    black, gray, white, red1, red2, orange, beige, yellow, green, light_green,
    cyan, light_blue, blue, purple, magenta, pink, brown
]

bridge = CvBridge()

def callback(imgmsg, pub):
    global parar_flag
    if parar_flag:
        return  # Ignora frames se estiver desativado

    try:
        frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")  # Converte msg ROS -> OpenCV
    except CvBridgeError:
        rospy.loginfo("Erro na conversão da imagem.")
        return

    height, width, _ = frame.shape
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color in colors_list:
        mask = cv2.inRange(hsv, color.lower, color.upper)
        if mask[height // 2][width // 2] == 255:
            pub.publish(color.name)
            rospy.loginfo(f"Cor detectada: {color.name}")
            break

def llacfront(data):
    global parar_flag
    if data.data == 'COR':
        parar_flag = False
        rospy.loginfo(" Deteccao de cor ATIVADA.")
    elif data.data == 'DESATIVAR':
        parar_flag = True
        rospy.loginfo(" Deteccao de cor DESATIVADA.")
    else:
        rospy.loginfo(f"Comando desconhecido: {data.data}")

def detect_color():
    rospy.init_node('detect_color')

    pub = rospy.Publisher('cor_detectada', String, queue_size=10)
    rospy.Subscriber('comandos', String, llacfront)
    rospy.Subscriber('frame', Image, callback, callback_args=pub)

    rospy.loginfo(" No detect_color iniciado. Aguardando comandos...")
    rospy.spin()


if __name__ == '__main__':
    try:
        detect_color()
    except rospy.ROSInterruptException:
        pass
