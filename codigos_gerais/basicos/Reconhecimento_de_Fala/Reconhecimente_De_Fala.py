import speech_recognition as sr
import time

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
                
                if texto_transcrito:
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

    