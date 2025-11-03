#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String

# Lista de palavras reconhecidas
lista = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco', '']

# Função principal
def hearing():
    pub = rospy.Publisher('words', String, queue_size=10)
    rospy.init_node('word_pub')
    rate = rospy.Rate(20)

    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source, duration=1)
        rospy.loginfo("Pronto! Fale agora...")

        # Loop de escuta
        while not rospy.is_shutdown():
            try:
                audio = r.listen(source, timeout=5, phrase_time_limit=5)
                texto_transcrito = (r.recognize_google(audio, language="pt-BR")).split()

                # Loop de avaliacao da frase
                for i in texto_transcrito:
                    encontrou = False
                    for palavra in lista:
                        if(i.lower() == palavra):
                            rospy.loginfo("\nPalavra chave encontrada: " + palavra)
                            pub.publish(palavra)
                            encontrou = True
                            break
                    if encontrou:
                        break
                if not encontrou:
                    rospy.loginfo("\nFrase aleatória")

            # Tabela de erros :)
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
        hearing()
    except rospy.ROSInterruptException:
        pass