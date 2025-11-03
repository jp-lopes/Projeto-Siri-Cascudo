#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

lista = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco']

def callback(data, pub):
    frase = (data.data).split()
    for i in frase:
        for palavra in lista:
            if(i.lower() == palavra):
                rospy.loginfo("Palavra chave detectada: " + palavra)
                pub.publish(palavra)
                return

def answer():
    rospy.init_node('keyword_pub')

    pub = rospy.Publisher('keyword', String, queue_size=10)

    rospy.Subscriber('frases', String, callback, callback_args=pub)

    rospy.spin()

if __name__ == '__main__':
    answer()