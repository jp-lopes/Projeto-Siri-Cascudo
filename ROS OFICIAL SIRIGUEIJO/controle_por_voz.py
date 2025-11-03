#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

lista_comandos = ['ativar camera', 'desativar camera', 
            'ativar voz', 'desativar voz', 
            'ativar movimentos', 'desativar movimentos']

def callback(data, pub):
    frase = data.data
    if frase in lista_comandos:
        rospy.loginfo("Comando detectado: " + frase.upper())
        pub.publish(frase.upper()) #publica com letra maiscula

def detectar_comando():
    rospy.init_node('controle_por_voz')
    pub = rospy.Publisher('comando_detectado', String, queue_size=10)
    rospy.Subscriber('frases', String, callback, callback_args=pub)
    rospy.spin()

if __name__ == '__main__':
    detectar_comando()