#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from unidecode import unidecode

from vosk import Model, KaldiRecognizer
import pyaudio
import json

class VozNode:
    def __init__(self):
        rospy.init_node('controle_por_voz')

        default_map = {
            'camera': 'CAMERA_CMD',
            'cores': 'CORES_CMD',
            'gestos': 'GESTOS_CMD',
            'desativar': 'DESATIVAR_CMD',
            'dinheiro': 'KEY_CMD',
            'plankton': 'PLANKTON_CMD',
            'plancton': 'PLANKTON_CMD',
            'lula': 'LULA_CMD',
            'molusco': 'LULA_CMD',
            'caramba': 'KEY_CMD'
        }
        
        self.mapa_comandos = rospy.get_param('~mapa_comandos', default_map)
        self.model_path = rospy.get_param('~model_path', '/home/jplop/model-pt')

        self.pub = rospy.Publisher('comandos', String, queue_size=10)

        rospy.loginfo("Carregando modelo Vosk de: " + self.model_path)
        
        self.model = None
        self.audio = None
        self.stream = None

        try:
            self.model = Model(self.model_path)
        except Exception as e:
            rospy.logerr(f"Falha ao carregar modelo Vosk: {self.model_path}")
            rospy.logerr(f"Erro: {e}")
            rospy.signal_shutdown("Erro no modelo Vosk")
            return

        self.rec = KaldiRecognizer(self.model, 16000)

        try:
            self.audio = pyaudio.PyAudio()
            self.stream = self.audio.open(format=pyaudio.paInt16,
                                          channels=1,
                                          rate=16000,
                                          input=True,
                                          frames_per_buffer=4096)
            self.stream.start_stream()
        except Exception as e:
            rospy.logerr(f"Falha ao abrir stream de áudio (PyAudio).")
            rospy.logerr(f"Erro: {e}")
            rospy.signal_shutdown("Erro no PyAudio")
            return

        rospy.loginfo("Modelo carregado. Pronto para ouvir!")


    def detectarComandosDeVoz(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            try:
                data = self.stream.read(4096, exception_on_overflow=False)

                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    texto = result.get("text", "")

                    if texto:
                        texto_processado = unidecode(texto.lower())
                        rospy.loginfo("Reconhecido: " + texto_processado)

                        publicado = False
                        
                        if texto_processado in self.mapa_comandos:
                            comando_publicar = self.mapa_comandos[texto_processado]
                            self.pub.publish(comando_publicar)
                            rospy.loginfo(f"Frase '{texto_processado}' ativou: {comando_publicar}")
                            publicado = True
                        
                        if not publicado:
                            for palavra in texto_processado.split():
                                if palavra in self.mapa_comandos:
                                    comando_publicar = self.mapa_comandos[palavra]
                                    self.pub.publish(comando_publicar)
                                    rospy.loginfo(f"Palavra '{palavra}' ativou: {comando_publicar}")
                                    break
            
            except IOError as e:
                rospy.logwarn(f"Erro de I/O no stream: {e}")
            
            rate.sleep()

    def cleanup(self):
        rospy.loginfo("Encerrando nó e fechando stream de áudio...")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
        rospy.loginfo("Stream fechado.")

if __name__ == '__main__':
    controleVoz = None
    try:
        controleVoz = VozNode()
        if not rospy.is_shutdown(): 
            controleVoz.detectarComandosDeVoz()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupção do ROS recebida.")
    except Exception as e:
        rospy.logerr(f"Erro inesperado no main: {e}")
    finally:
        if controleVoz:
            controleVoz.cleanup()