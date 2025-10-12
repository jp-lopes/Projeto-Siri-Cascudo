#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String

def asking():
    pub = rospy.Publisher('frases', String, queue_size=10)
    rospy.init_node('Ouvindo_vozes')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            rospy.loginfo("Ajustando para o ruído ambiente. Por favor, aguarde...")
            r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Pronto! Fale agora...")
            
            while True:
                try:
                    audio = r.listen(source, timeout=5, phrase_time_limit=5)
                    
                    texto_transcrito = r.recognize_google(audio, language="pt-BR")
                    
                    if texto_transcrito:
                        rospy.loginfo(f"Você disse: {texto_transcrito}")
                        pub.publish(texto_transcrito)
                    
                except sr.WaitTimeoutError:
                    print("Esperando você falar...")
                except sr.UnknownValueError:
                    print("Não entendi o que você disse.")
                except sr.RequestError as e:
                    print(f"Erro na requisição; {e}")
                except Exception as e:
                    print(f"Ocorreu um erro: {e}")
            
        rate.sleep()
        
if __name__ == '__main__':
    try:
        asking()
    except rospy.ROSInterruptException:
        pass