#tentei explicar o q cada coisa faz o melhor possível, mas se tiver dúvida me pergunte! perdão pela demora, tá difícil escrever sem óculos :(
import speech_recognition as sr
import time
from gtts import gTTS
import os
from playsound import playsound
import threading
import sys

# variáveis globais para controle
stop_event = threading.Event()
keyboard_available = False
keyboard = None

# tenta importar a biblioteca 'keyboard' e define a flag de disponibilidade.
try:
    import keyboard  # type: ignore
    keyboard_available = True
except ImportError:
    pass # fallback
def start_stop_listener():
    """
    Inicia uma thread separada para escutar o pressionar da tecla 'q'
    e definir o stop_event quando detectada.
    """
    def listener():
        # se o módulo 'keyboard' estiver disponível, use detecção sem Enter.
        if keyboard_available:
            try:
                print("era pra funcionar, mas só funfa com ctrl c")
                while not stop_event.is_set():
                    if keyboard.is_pressed('q'):
                        print("\nInterrompendo...")
                        stop_event.set()
                        break
                    # pausa para evitar alto uso de CPU
                    time.sleep(0.1)
                return # sai da função interna
            except Exception:
                # se ocorrer qualquer erro com 'keyboard' (permissões, etc.),
                # a execução cairá no fallback abaixo.
                print("Aviso: Falha na escuta 'q' sem Enter.")
        
        # fallback caso a biblioteca 'keyboard' falhe ou não esteja disponível
        # apenas executa se o stop_event ainda não tiver sido definido pela tentativa acima.
        if not stop_event.is_set():
            print("Pressione 'q' e Enter para parar.")
            while not stop_event.is_set():
                # lê a entrada do terminal
                # sys.stdin.readline evita bo de buffer
                line = sys.stdin.readline().strip().lower()
                if line == 'q':
                    print("Interrompendo...")
                    stop_event.set()
                    break

    # inicia a thread em modo 'daemon' pra que ela termine quando o programa principal terminar
    t = threading.Thread(target=listener, daemon=True)
    t.start()


def transcrever_audio_em_tempo_real():
    """
    Função principal que lida com a captura e reconhecimento de fala.
    """
    r = sr.Recognizer()

    # inicia a escuta da tecla de parada
    start_stop_listener()

    try:
        with sr.Microphone() as source:
            print("Ajustando para o ruído ambiente. Por favor, aguarde...")
            # ajusta dinamicamente a sensibilidade do microfone ao ruído de fundo
            r.adjust_for_ambient_noise(source, duration=1)
            print("Pronto! Fale agora... (pressione 'q' para parar)")

            # loop principal de reconhecimento de fala
            while not stop_event.is_set():
                try:
                    # ouve o áudio. Tenta ouvir por no máximo 5 segundos ou até uma pausa de 5 segundos
                    audio = r.listen(source, timeout=5, phrase_time_limit=5)

                    # tenta transcrever usando o Google Speech Recognition (requer internet)
                    texto_transcrito = r.recognize_google(audio, language="pt-BR")
                    texto_lower = texto_transcrito.lower()

                    # verifica por palavras-chave específicas e responde
                    if "dinheiro" in texto_lower:
                        print(f"Você disse: {texto_transcrito}")
                        # toca o som do sirigueijo
                        caminho_sirigueijo = '/home/fer/Downloads/sirigueijo-eu-gosto-de-dinheiro.mp3'
                        if os.path.exists(caminho_sirigueijo):
                             playsound(caminho_sirigueijo)
                        
                    elif "siri cascudo" in texto_lower:
                        print(f"Comando detectado: {texto_transcrito}")
                        tts = gTTS(text="trabalho", lang="pt")
                        tts.save("trab_tmp.mp3")
                        playsound("trab_tmp.mp3")
                        os.remove("trab_tmp.mp3")
                        
                    elif "bob esponja" in texto_lower:
                        print(f"Comando detectado: {texto_transcrito}")
                        tts = gTTS(text="bom esponja", lang="pt")
                        tts.save("esponja_tmp.mp3")
                        playsound("esponja_tmp.mp3")
                        os.remove("esponja_tmp.mp3")
                        
                    elif "lula molusco" in texto_lower:
                        print(f"Comando detectado: {texto_transcrito}")
                        tts = gTTS(text="minhas bolas", lang="pt") 
                        tts.save("lula_tmp.mp3")
                        playsound("lula_tmp.mp3")
                        os.remove("lula_tmp.mp3")
                        
                    elif "nematelmintos" in texto_lower:
                        print(f"Comando detectado: {texto_transcrito}")
                        tts = gTTS(text="nossa que pinto enorme", lang="pt") 
                        caminho_nematelmintos = '/home/fer/Downloads/caramba.mp3'
                        if os.path.exists(caminho_nematelmintos):
                             playsound(caminho_nematelmintos)

                    else:
                        # se nenhuma palavra-chave for detectada, apenas imprime a transcrição
                        print(f"Você disse: {texto_transcrito}")

                except sr.WaitTimeoutError:
                    if not stop_event.is_set():
                        print("Esperando você falar...")
                except sr.UnknownValueError:
                    # o google não conseguiu entender o áudio
                    if not stop_event.is_set():
                        print("Não entendi o que você disse.")
                except sr.RequestError as e:
                    # problema de conexão ou API
                    print(f"Erro na requisição ao serviço de reconhecimento; {e}")
                except Exception as e:
                    # outras exceções (ex: problema no playsound/gTTS)
                    print(f"Ocorreu um erro: {e}")

                # pausa antes de começar a ouvir novamente
                if not stop_event.is_set():
                    time.sleep(1.0)
                    
    except KeyboardInterrupt:
        # permite parar o programa com Ctrl+C
        print("\nInterrupção por teclado (Ctrl+C).")
    except Exception as e:
        # exceção na inicialização (ex: microfone não encontrado)
        print(f"\nErro fatal ao iniciar o microfone: {e}")
        
    print("Programa finalizado.")

if __name__ == "__main__":
    transcrever_audio_em_tempo_real()