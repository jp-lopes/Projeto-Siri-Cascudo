import speech_recognition as sr
import time
from gtts import gTTS
import os
from playsound import playsound

def transcrever_audio_em_tempo_real():
    r = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("Ajustando para o ruído ambiente. Por favor, aguarde...")
        r.adjust_for_ambient_noise(source, duration=1)
        print("Pronto! Fale agora...")
        
        while True:
            try:
                audio = r.listen(source, timeout=5, phrase_time_limit=5)
                
                texto_transcrito = r.recognize_google(audio, language="pt-BR")
                
                if "dinheiro" in texto_transcrito.lower():
                    print("dinheiro")
                    
                    # Cria e salva o áudio da palavra "dinheiro"
                    tts = gTTS(text="dinheiro", lang="pt")
                    tts.save("dinheiro.mp3")
                    
                    # Reproduz o arquivo de áudio
                    playsound("dinheiro.mp3")
                    
                    # Opcional: remove o arquivo de áudio para não poluir a pasta
                    os.remove("dinheiro.mp3")
                else:
                    print(f"Você disse: {texto_transcrito}")
                
            except sr.WaitTimeoutError:
                print("Esperando você falar...")
            except sr.UnknownValueError:
                print("Não entendi o que você disse.")
            except sr.RequestError as e:
                print(f"Erro na requisição; {e}")
            except Exception as e:
                print(f"Ocorreu um erro: {e}")
            
            time.sleep(0.5)

if __name__ == "__main__":
    transcrever_audio_em_tempo_real()