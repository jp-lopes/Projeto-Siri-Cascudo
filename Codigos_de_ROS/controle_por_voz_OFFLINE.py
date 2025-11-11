#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String
from unidecode import unidecode

from vosk import Model, KaldiRecognizer
import pyaudio
import json

#Offline porem meio burro

comandos = ['ativar camera', 'ativar a camera', 'ativar cor', 'ativar gestos', 'desativar tudo']
palavras_chave = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco', 'platelminto']

class VozNode:
    def __init__(self):
        rospy.init_node('controle_por_voz')
        self.pub = rospy.Publisher('comandos', String, queue_size=10)

        rospy.loginfo("Carregando modelo Vosk...")
        self.model = Model("/home/jplop/model-pt")
        self.rec = KaldiRecognizer(self.model, 16000)

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=pyaudio.paInt16,
                                      channels=1,
                                      rate=16000,
                                      input=True,
                                      frames_per_buffer=4096)
        self.stream.start_stream()
        rospy.loginfo("Modelo carregado. Pronto para ouvir!")


    def detectarComandosDeVoz(self):
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                data = self.stream.read(4096, exception_on_overflow=False)

                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    texto = result.get("text", "")

                    if texto:
                        texto_processado = unidecode(texto.lower())
                        rospy.loginfo("Reconhecido: " + texto_processado)

                        # Detectar comandos exatos
                        if texto_processado in comandos:
                            self.pub.publish(texto_processado.upper())
                            rospy.loginfo("Comando detectado: " + texto_processado.upper())

                        # Palavras chave
                        for palavra in texto_processado.split():
                            if palavra in palavras_chave:
                                self.pub.publish(palavra)
                                rospy.loginfo("Palavra chave detectada: " + palavra)
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