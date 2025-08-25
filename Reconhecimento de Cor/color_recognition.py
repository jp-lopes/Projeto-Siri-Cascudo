import cv2
import numpy as np
import colors

cap = cv2.VideoCapture(0)

while True:
    #Leitura do vídeo
    ret,frame = cap.read()
    if ret==False:
        print("Error")
        break
    width = int(cap.get(3))
    height = int(cap.get(4))
    
    #Processamento do frame de vídeo
    frame = cv2.flip(frame,1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.rectangle(frame,(width//2-1,height//2+1),(width//2+1,height//2-1),(0,255,0),1,cv2.LINE_AA)

    for color in colors.colors_list:
        mask = cv2.inRange(hsv, color.lower, color.upper)
        if mask[height//2][width//2] == 255:
            frame = cv2.putText(frame,color.name,(width//2-35,height-50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),1,cv2.LINE_AA)
            break

    #Exibir o frame processado na tela
    cv2.imshow("frame",frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()