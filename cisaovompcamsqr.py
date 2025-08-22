import cv2
import numpy as np

def get_color_name(bgr):
    
    bgr_np = np.uint8([[bgr]])
    hsv = cv2.cvtColor(bgr_np, cv2.COLOR_BGR2HSV)[0][0]
    h, s, v = hsv

    if v < 50:
        return 'Black'
    if s < 50 and v > 200:
        return 'White'
    if s < 50:
        return 'Gray'
    if h < 10 or h >= 170:
        return 'Red'
    if 10 <= h < 25:
        return 'Orange'
    if 25 <= h < 35:
        return 'Yellow'
    if 35 <= h < 85:
        return 'Green'
    if 85 <= h < 125:
        return 'Cyan'
    if 125 <= h < 150:
        return 'Blue'
    if 150 <= h < 170:
        return 'Magenta'
    return 'Unknown'

def identify_color_realtime():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open webcam")
    print("Press 'q' to quit.")

    square_size = 20  #tamanho do quadrado

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        half = square_size // 2

    
        cv2.rectangle(frame, (cx - half, cy - half), (cx + half, cy + half), (255, 255, 255), 2)

     
        roi = frame[cy - half:cy + half, cx - half:cx + half]
        avg_color = cv2.mean(roi)[:3]
        avg_color_bgr = tuple(map(int, avg_color))
        color_name = get_color_name(avg_color_bgr)

        cv2.putText(frame, f'Color: {color_name}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv2.LINE_AA)

        cv2.imshow('Color Identifier', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

identify_color_realtime()