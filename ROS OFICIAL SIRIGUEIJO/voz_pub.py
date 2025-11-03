#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String
from unidecode import unidecode

def asking():
    pub = rospy.Publisher('frases', String, queue_size=10)
    rospy.init_node('voz_pub')
    rate = rospy.Rate(20)
    
    try:
        while not rospy.is_shutdown():
            r = sr.Recognizer()
            with sr.Microphone() as source:
                rospy.loginfo("Ajustando para o ruído ambiente. Por favor, aguarde...")
                r.adjust_for_ambient_noise(source, duration=1)
                rospy.loginfo("Pronto! Fale agora...")
                
                while not rospy.is_shutdown():
                    try:
                        audio = r.listen(source, timeout=5, phrase_time_limit=5)
                        
                        texto_transcrito = r.recognize_google(audio, language="pt-BR")
                        
                        if texto_transcrito:
                            texto_processado = unidecode(texto_transcrito) # remove acentos da string
                            texto_processado = texto_processado.lower() # transforma para letra minuscula
                            rospy.loginfo(f"Você disse: {texto_processado}")
                            pub.publish(texto_processado)
                        
                    except sr.WaitTimeoutError:
                        rospy.loginfo("Esperando você falar...")
                    except sr.UnknownValueError:
                        rospy.loginfo("Não entendi o que você disse.")
                    except sr.RequestError as e:
                        rospy.loginfo(f"Erro na requisição; {e}")
                    except Exception as e:
                        rospy.loginfo(f"Ocorreu um erro: {e}")
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Encerrando nó...")
        
if __name__ == '__main__':
    try:
        asking()
    except rospy.ROSInterruptException:
        pass