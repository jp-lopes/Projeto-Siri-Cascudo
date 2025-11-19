#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os

def callback(data):
    if data.data == 'dinheiro':
        os.system("mpg123 /home/sirigueijo/siricascudo_ws/src/aborgue/scripts/dinheiro.mp3")

def playaudio():
    rospy.init_node("audios")
    rospy.Subscriber("comandos", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        playaudio()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Encerrando nó...")
