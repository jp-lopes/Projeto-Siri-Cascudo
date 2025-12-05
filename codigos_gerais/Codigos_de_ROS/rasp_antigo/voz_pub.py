#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String

# Variável de parada
parar_flag = True

# Lista de palavras reconhecidas
lista = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco']

# Função do subscriber
def callback(data):
    global parar_flag
    if data.data == 'VOZ':
        parar_flag = False
        rospy.loginfo("Detecção de VOZ ATIVADA.")
    elif data.data == 'DESATIVAR':
        parar_flag = True
        rospy.loginfo("Detecção de VOZ DESATIVADA.")
    else:
        rospy.loginfo(f"Comando desconhecido: {data.data}")

# Função principal
def hearing():
    global parar_flag
    pub = rospy.Publisher('words', String, queue_size=10)
    rospy.init_node('word_pub')
    rospy.Subscriber('comandos', String, callback)
    rate = rospy.Rate(1)

    r = sr.Recognizer()
    mic = sr.Microphone()

    rospy.loginfo("Nó de detecção de voz iniciado. Aguardando comando 'VOZ'...")

    while not rospy.is_shutdown():
        if parar_flag:
            rate.sleep()
            continue

        with mic as source:
            r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Pronto! Fale agora...")

            try:
                audio = r.listen(source, timeout=5, phrase_time_limit=5)
                texto_transcrito = (r.recognize_google(audio, language="pt-BR")).split()

                encontrou = False
                for i in texto_transcrito:
                    for palavra in lista:
                        if i.lower() == palavra:
                            rospy.loginfo(f"\nPalavra-chave encontrada: {palavra}")
                            pub.publish(palavra)
                            encontrou = True
                            break
                    if encontrou:
                        break

                if not encontrou:
                    rospy.loginfo("Frase aleatória.")

            except sr.WaitTimeoutError:
                rospy.loginfo("Esperando você falar...")
            except sr.UnknownValueError:
                rospy.loginfo("Não entendi o que você disse.")
            except sr.RequestError as e:
                rospy.logwarn(f"Erro na requisição: {e}")
            except Exception as e:
                rospy.logerr(f"Ocorreu um erro: {e}")

        rate.sleep()


if __name__ == '__main__':
    try:
        hearing()
    except rospy.ROSInterruptException:
        pass
