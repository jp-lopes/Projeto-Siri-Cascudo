import numpy as np
import cv2

dicio = {'black' : [0, 0, 0],
         'white' : [219,247,255],
         'red' : [0, 0, 255],
         'green' : [0, 200, 0],
         'blue' : [255, 0, 0],
         'beige' : [148,205,255],
         'brown' : [37,62,107]}



def closer (point, dicio):
    best_eval = int(1000)
    for i in dicio:
        evaluation = abs((dicio[i][0] - point[0])) + abs((dicio[i][1] - point[1])) + abs((dicio[i][2] - point[2]))
        if evaluation < best_eval:
            best_eval = evaluation
            color = dicio[i]
    color = reverse_search(dicio, color)
    return color

def reverse_search(di, val):
    for key in di:
        if (di[key] == val):
            return key
    raise LookupError('No match found')

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    height, width, _ = frame.shape

    point = frame[int(height/2), int(width/2)]

    #print(type(dicio['black'][0]))
    #print(type(np.array(point).tolist()[0]))

    point2 = np.array(point).tolist()

    color = closer(point2, dicio)

    font = cv2.FONT_HERSHEY_SIMPLEX 
    frame = cv2.circle(frame, ((width)//2,(height)//2), 6, (0,0,255), 2)
    frame = cv2.putText(frame, color, (int(height/2 - 10/2), int(width/2 - 10/2)), font, 1, (0,255,128), 3, cv2.LINE_AA)


    cv2.imshow('Image', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


