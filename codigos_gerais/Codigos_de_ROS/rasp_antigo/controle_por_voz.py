#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String
from unidecode import unidecode

comandos = ['camera', 'cor', 'gestos', 'desativar', 'voz']
palavras_chave = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco', 'platelminto']

class VozNode:
    def __init__(self):
        self.r = sr.Recognizer()


    def detectarComandosDeVoz(self):
        pub = rospy.Publisher('comandos', String, queue_size=10)
        rospy.init_node('controle_por_voz')
        rate = rospy.Rate(20)

        with sr.Microphone() as source:
            rospy.loginfo("Ajustando para o ruído ambiente. Por favor, aguarde...")
            self.r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Pronto! Fale agora...")
            
            try:
                while not rospy.is_shutdown():
                    try:
                        audio = self.r.listen(source, timeout=2, phrase_time_limit=3)
                        texto_transcrito = self.r.recognize_google(audio, language="pt-BR")
                    except sr.WaitTimeoutError:
                        pass
                    except sr.UnknownValueError:
                        pass
                    except sr.RequestError as e:
                        rospy.loginfo(f"Erro na requisição; {e}")
                    except Exception as e:
                        rospy.loginfo(f"Ocorreu um erro: {e}")
                    else:
                        if texto_transcrito:
                            texto_processado = unidecode(texto_transcrito) # remove acentos da string
                            texto_processado = texto_processado.lower() # transforma para letra minuscula

                            if texto_processado in comandos:
                                rospy.loginfo("Comando detectado: " + texto_processado.upper())
                                pub.publish(texto_processado.upper()) #publica com letra maiscula
                            
                            frase = texto_processado.split()
                            for palavra in frase:
                                if palavra in palavras_chave:
                                    rospy.loginfo("Palavra chave detectada: " + palavra)
                                    pub.publish(palavra)
                                    break
                    rate.sleep()
            except KeyboardInterrupt:
                pass
            finally:
                rospy.loginfo("Encerrando nó...")
        
if __name__ == '__main__':
    try:
        controleVoz = VozNode()
        controleVoz.detectarComandosDeVoz()
    except rospy.ROSInterruptException:
        pass