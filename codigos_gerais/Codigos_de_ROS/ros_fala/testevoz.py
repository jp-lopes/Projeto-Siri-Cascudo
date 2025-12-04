#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from unidecode import unidecode
import pyaudio
import numpy as np
import webrtcvad
import deepspeech

# Comandos e palavras-chave (iguais ao seu)
comandos = ['camera', 'cor', 'gestos', 'desativar']
palavras_chave = ['dinheiro', 'plankton', 'plancton', 'lula', 'molusco', 'platelminto']

# --- Configurações do DeepSpeech e VAD ---

# ATENÇÃO: Ajuste estes caminhos para seus arquivos .pbmm e .scorer
DEEPSPEECH_MODEL = "/home/jplop/model-pt/deepspeech-pt.pbmm" # Ex: deepspeech-0.9.3-models.pbmm
DEEPSPEECH_SCORER = "/home/jplop/model-pt/deepspeech-pt.scorer" # Ex: deepspeech-0.9.3-models.scorer

# Configurações do VAD (detector de atividade de voz)
VAD_AGGRESSIVENESS = 3   # De 0 (menos agressivo) a 3 (mais agressivo)
VAD_FRAME_MS = 30        # Duração de cada "chunk" de áudio que o VAD analisa (30ms)
VAD_CHUNK_SIZE = int(16000 * VAD_FRAME_MS / 1000) # = 480 frames (amostras)

# --- Fim das Configurações ---


class VozNode:
    def __init__(self):
        rospy.init_node('controle_por_voz')
        self.pub = rospy.Publisher('comandos', String, queue_size=10)

        rospy.loginfo("Aviso: O projeto DeepSpeech da Mozilla foi descontinuado.")
        rospy.loginfo("Carregando modelo DeepSpeech...")
        
        try:
            self.model = deepspeech.Model(DEEPSPEECH_MODEL)
            self.model.enableExternalScorer(DEEPSPEECH_SCORER)
        except RuntimeError as e:
            rospy.logerr(f"Erro ao carregar o modelo DeepSpeech: {e}")
            rospy.logerr(f"Verifique se os caminhos '{DEEPSPEECH_MODEL}' e '{DEEPSPEECH_SCORER}' estão corretos.")
            return

        # Inicializa o VAD
        self.vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
        
        # Inicializa o stream do DeepSpeech
        self.ds_stream = self.model.createStream()
        self.speaking = False

        # Configura o PyAudio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=pyaudio.paInt16,
                                      channels=1,
                                      rate=16000,
                                      input=True,
                                      # IMPORTANTE: O VAD precisa de chunks de 30ms (480 frames)
                                      frames_per_buffer=VAD_CHUNK_SIZE)
        
        self.stream.start_stream()
        rospy.loginfo("Modelo carregado. Pronto para ouvir!")


    def detectarComandosDeVoz(self):
        rate = rospy.Rate(20)
        
        try:
            while not rospy.is_shutdown():
                
                frame = self.stream.read(VAD_CHUNK_SIZE, exception_on_overflow=False)
                is_speech = self.vad.is_speech(frame, 16000)

                if is_speech and not self.speaking:

                    rospy.loginfo("Detectei fala...")
                    self.speaking = True

                if self.speaking:

                    frame_np = np.frombuffer(frame, dtype=np.int16)
                    self.ds_stream.feedAudioContent(frame_np)

                if not is_speech and self.speaking:
                    # Parou de falar
                    rospy.loginfo("Fala terminada, processando...")
                    self.speaking = False
                    
                    # Finaliza o stream e obtém o texto
                    texto = self.ds_stream.finishStream()
                    
                    # Cria um novo stream para a próxima vez
                    self.ds_stream = self.model.createStream() 

                    if texto:
                        texto_processado = unidecode(texto.lower())
                        rospy.loginfo("Reconhecido: " + texto_processado)

                        # Sua lógica de palavras-chave
                        palavras_encontradas = texto_processado.split()
                        
                        # Verifica se o comando inteiro foi dito
                        if texto_processado in comandos:
                            self.pub.publish(texto_processado)
                            rospy.loginfo("Comando detectado: " + texto_processado)
                        else:
                            # Verifica se alguma palavra-chave foi dita
                            for palavra in palavras_encontradas:
                                if palavra in palavras_chave:
                                    self.pub.publish(palavra)
                                    rospy.loginfo("Palavra chave detectada: " + palavra)
                                    break # Publica só a primeira que encontrar

                rate.sleep()
        
        except KeyboardInterrupt:
            pass
        finally:
            rospy.loginfo("Encerrando nó...")
            self.stream.stop_stream()
            self.stream.close()
            self.audio.terminate()
        
if __name__ == '__main__':
    try:
        controleVoz = VozNode()
        controleVoz.detectarComandosDeVoz()
    except rospy.ROSInterruptException:
        pass