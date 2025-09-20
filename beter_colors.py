# Autores: Igor de Moura Fonseca e Guilherme de Campos Silva

# Para melhorar nosso código de detecção de cor decidimos seguir uma análise vetorial das variáveis vermelho, verde e azul
    # Com isso, adotamos a ideia de que esse espaço vetorial é um cubo que parte do (0,0,0) e termina em seu vertice oposto (255,255,255)
    # Com isso em mente, criamos essa classe que pega um pixel da escala BGR e o destrincha em diferentes parâmetros
        # módulo - tamanho do vetor                         ((B^2 + G^2 + R^2)^0.5)
        # theta - angulo entre a reta verde e a vermelha    (arctan(G/R) * 180/pi)
        # phi - angulo entre a reta azul e a vermelha       (arctan(B/R) * 180/pi)
        # sigma - angulo entre a reta azul e a verde        (arctan(B/G) * 180/pi)

#------------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------

import numpy as np
import cv2
from math import atan, pi

class coor:

    def __init__(self, pixel):         
        self.modulo = int((pixel[0]**2 + pixel[1]**2 + pixel[2]**2)**0.5)
        self.theta = atan((pixel[1]/(pixel[2]+0.01))) * 180/pi
        self.phi = atan((pixel[0]/(pixel[2]+0.01))) * 180/pi
        self.sigma = atan((pixel[0]/(pixel[1]+0.01))) * 180/pi

    # A origem desse espaço de cores é a região do preto, então todo vetor pequeno terá uma tonalidade bem escura
    def preto(self):
        if (self.modulo < 50):
            return 'Preto'
    
    # A diagonal principal do cubo de cores consiste da transformação do preto pro branco
    def cinza_e_branco(self):
        if (self.theta > 40 and self.theta < 50 and self.phi > 40 and self.phi < 50 and self.sigma > 40 and self.sigma < 50):
            if (self.modulo > 270):
                return 'Branco'
            else:
                return 'Cinza'

    # A escolha dos parametros das proximas cores foi obtida de maneira empirica, anotando os dados obtidos ao observar diferentes tonalidades de cada cor
        # Para fazer analises, basta imprimir um objeto desta classe e ver quais são os maximos e minimos alcançados pelos angulos

    def azul(self):
        if (self.theta > 50 and self.theta < 66 and self.phi > 50 and self.phi < 80 and self.sigma > 50 and self.sigma < 80):
            return 'Azul'
        
    def vermelho(self):
        if (self.theta > 8 and self.theta < 25 and self.phi > 11 and self.phi < 40 and self.sigma > 42 and self.sigma < 63):
            return 'Vermelho'
        
    def verde(self):
        if (self.theta > 45 and self.theta < 71 and self.phi > 21 and self.phi < 60 and self.sigma > 12 and self.sigma < 40):
            return 'Verde'
        
    def amarelo(self):
        if (self.theta > 35 and self.theta < 47 and self.phi > 8 and self.phi < 25 and self.sigma > 13 and self.sigma < 27):
            return 'Amarelo'
    
    # Loop que passa por cada cor definida nessa classe
    def escolha(self):
        # chama cada função e retorna a primeira que não for None
        for detector in (self.preto, self.cinza_e_branco, self.azul, self.vermelho, self.verde, self.amarelo):
            cor = detector()
            if cor is not None:
                return cor
        return None

    # Função de impressão
    def __str__(self):
        strin = "({}, {}, {}, {})".format(self.modulo, int(self.theta), int(self.phi), int(self.sigma))
        return strin


# Código clássico de visão computacional
# Válido notar que só está sendo analisado o pixel central da câmera
cap = cv2.VideoCapture(0)
while True:

    ret, frame = cap.read()
    height, width, _ = frame.shape

    point = frame[int(height/2), int(width/2)]

    point2 = np.array(point).tolist()

    color = coor(point2)

    font = cv2.FONT_HERSHEY_SIMPLEX 
    frame = cv2.circle(frame, ((width)//2,(height)//2), 6, (0,0,255), 2)
    frame = cv2.putText(frame, color.escolha(), (int(height/2 - 10/2), int(width/2 - 10/2)), font, 1, (32,94,27), 3, cv2.LINE_AA)
    print(color)

    cv2.imshow('Image', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()