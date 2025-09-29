#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

lista = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco']

def callback(data, pub):
    frase = (data.data).split()
    for i in frase:
        for palavra in lista:
            if(i.lower() == palavra):
                rospy.loginfo("\nPalavra chave encontrada: " + palavra)
                palavra_chave = palavra
                pub.publish(palavra_chave)
                return
        rospy.loginfo("\nfrase aleatória")

def answer():
    rospy.init_node('Banco_de_palavras')

    pub = rospy.Publisher('Palavra_chave', String, queue_size=10)

    rospy.Subscriber('frases', String, callback, callback_args=pub)

    rospy.spin()

if __name__ == '__main__':
    answer()