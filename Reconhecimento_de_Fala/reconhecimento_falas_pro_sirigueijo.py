import cv2
import numpy as np
import pyttsx3

#Inicializa o motor de fala e define um controle pra fala
engine = pyttsx3.init()
encontrado_dinheiro = False
encontrado_bob_esponja = False

def reconhecer_objetos(frame):
    global encontrado_dinheiro, encontrado_bob_esponja
    
    #Converte o frame da cam pro espaço de cores em HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #Detecção de Dinheiro (Verde)
    cor_verde_baixo = np.array([40, 50, 50])
    cor_verde_alto = np.array([80, 255, 255])
    
    mascara_verde = cv2.inRange(hsv, cor_verde_baixo, cor_verde_alto)
    contornos_verde, _ = cv2.findContours(mascara_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    dinheiro_detectado_no_frame = False
    for contorno in contornos_verde:
        perimetro = cv2.arcLength(contorno, True)
        aproximacao = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
        
        if len(aproximacao) == 4:
            x, y, w, h = cv2.boundingRect(aproximacao)
            if w > 50 and h > 50:
                proporcao = float(w) / h
                if proporcao > 1.2 and proporcao < 1.8:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, "Dinheiro", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    dinheiro_detectado_no_frame = True
    
    #Lógica de fala pra dinheiro
    if dinheiro_detectado_no_frame and not encontrado_dinheiro:
        engine.say("Dinheiro")
        engine.runAndWait()
        encontrado_dinheiro = True
    elif not dinheiro_detectado_no_frame:
        encontrado_dinheiro = False

    #Detecção do Bob Esponja
    cor_amarela_baixo = np.array([20, 100, 100])
    cor_amarela_alto = np.array([40, 255, 255])
    
    mascara_amarela = cv2.inRange(hsv, cor_amarela_baixo, cor_amarela_alto)
    contornos_amarela, _ = cv2.findContours(mascara_amarela, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    bob_esponja_detectado_no_frame = False
    for contorno in contornos_amarela:
        perimetro = cv2.arcLength(contorno, True)
        aproximacao = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
        
        if len(aproximacao) == 4:
            x, y, w, h = cv2.boundingRect(aproximacao)
            if w > 50 and h > 50:
                proporcao = float(w) / h
                if proporcao > 0.8 and proporcao < 1.2:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, "Bob Esponja", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    bob_esponja_detectado_no_frame = True
    
    #Lógica de fala pra Bob Esponja
    if bob_esponja_detectado_no_frame and not encontrado_bob_esponja:
        engine.say("Bob Esponja")
        engine.runAndWait()
        encontrado_bob_esponja = True
    elif not bob_esponja_detectado_no_frame:
        encontrado_bob_esponja = False
        
    return frame

#Captura de vídeo
cap = cv2.VideoCapture(0)

if cap.isOpened():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame_processado = reconhecer_objetos(frame)
        
        cv2.imshow('Reconhecimento de Objetos', frame_processado)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()