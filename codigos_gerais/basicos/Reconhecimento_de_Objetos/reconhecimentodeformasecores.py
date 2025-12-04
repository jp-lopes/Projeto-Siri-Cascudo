import cv2
import numpy as np
import pyttsx3
import threading

engine = pyttsx3.init()

speech_lock = threading.Lock()

def falar_em_thread(texto):
    if speech_lock.acquire(blocking=False):
        try:
            engine.say(texto)
            engine.runAndWait()
        finally:
            speech_lock.release()

def processar_frame_cores(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    fala_para_dizer = None

    cor_verde_baixo = np.array([40, 50, 50])
    cor_verde_alto = np.array([80, 255, 255])
    mascara_verde = cv2.inRange(hsv, cor_verde_baixo, cor_verde_alto)
    
    contornos_verdes, _ = cv2.findContours(mascara_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contorno in contornos_verdes:
        perimetro = cv2.arcLength(contorno, True)
        aproximacao = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
        
        if len(aproximacao) == 4:
            x, y, w, h = cv2.boundingRect(aproximacao)
            
            if w > 50 and h > 50:
                proporcao = float(w) / h
                if proporcao > 1.2 and proporcao < 1.8:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, "Plankton (Verde)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    fala_para_dizer = "Plankton"
                    break

    if fala_para_dizer is None:
        cor_amarelo_baixo = np.array([20, 100, 100])
        cor_amarelo_alto = np.array([30, 255, 255])
        mascara_amarelo = cv2.inRange(hsv, cor_amarelo_baixo, cor_amarelo_alto)
        
        contornos_amarelos, _ = cv2.findContours(mascara_amarelo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contorno in contornos_amarelos:
            x, y, w, h = cv2.boundingRect(contorno)
            if w > 50 and h > 50:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, "Bob Esponja (Amarelo)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                fala_para_dizer = "Bob Esponja"
                break

    if fala_para_dizer is None:
        cor_vermelho_baixo1 = np.array([0, 100, 100])
        cor_vermelho_alto1 = np.array([10, 255, 255])
        cor_vermelho_baixo2 = np.array([170, 100, 100])
        cor_vermelho_alto2 = np.array([180, 255, 255])
        
        mascara_vermelho1 = cv2.inRange(hsv, cor_vermelho_baixo1, cor_vermelho_alto1)
        mascara_vermelho2 = cv2.inRange(hsv, cor_vermelho_baixo2, cor_vermelho_alto2)
        mascara_vermelho = cv2.bitwise_or(mascara_vermelho1, mascara_vermelho2)
        
        contornos_vermelhos, _ = cv2.findContours(mascara_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contorno in contornos_vermelhos:
            x, y, w, h = cv2.boundingRect(contorno)
            if w > 50 and h > 50:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "Seu Siriguejo (Vermelho)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                fala_para_dizer = "Seu Siriguejo"
                break

    if fala_para_dizer:
        t = threading.Thread(target=falar_em_thread, args=(fala_para_dizer,))
        t.daemon = True
        t.start()

    return frame

cap = cv2.VideoCapture(0)

if cap.isOpened():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame_processado = processar_frame_cores(frame)
        
        cv2.imshow('Reconhecimento de Cores', frame_processado)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()