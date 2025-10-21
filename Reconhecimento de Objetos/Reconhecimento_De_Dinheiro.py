import cv2
import numpy as np
import pyttsx3

engine = pyttsx3.init()

def reconhecer_dinheiro_verde(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    cor_verde_baixo = np.array([40, 50, 50])
    cor_verde_alto = np.array([80, 255, 255])
    
    mascara = cv2.inRange(hsv, cor_verde_baixo, cor_verde_alto)
    
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    dinheiro_encontrado = False
    
    for contorno in contornos:
        perimetro = cv2.arcLength(contorno, True)
        aproximacao = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
        
        if len(aproximacao) == 4:
            x, y, w, h = cv2.boundingRect(aproximacao)
            
            if w > 50 and h > 50:
                proporcao = float(w) / h
                if proporcao > 1.2 and proporcao < 1.8:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, "Nota de Dinheiro?", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    dinheiro_encontrado = True

    if dinheiro_encontrado:
        engine.say("Plankton")
        engine.runAndWait()

    return frame

cap = cv2.VideoCapture(0)

if cap.isOpened():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame_processado = reconhecer_dinheiro_verde(frame)
        
        cv2.imshow('Reconhecimento de Dinheiro', frame_processado)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()