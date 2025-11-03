
"Código de teste para entender a qualidade da cámera da rasp"
"Básicamente liga a cámera e tira uma foto"

import cv2

contador = 0
if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    # Loop pra dar tempo da cámera abrir
    while contador < 80:
        ret, frame = cap.read()
        if not ret:
            print("Frame inválido, tentando novamente...")
            continue

        contador += 1

        # Se der erro, provavelmente o path da file tá errado
        if contador == 78:
            cv2.imwrite('/home/sirigueijo/screenshot.jpg', frame)
            print("Imagem salva!")

    cap.release()
    cv2.destroyAllWindows()
