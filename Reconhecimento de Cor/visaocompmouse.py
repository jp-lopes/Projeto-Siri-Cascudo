import cv2
import numpy as np
def identify_color_realtime():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open webcam")
    print("Press 'q' to quit. Move mouse over the window to get color at cursor.")

    color_txt_window = np.zeros((100, 300, 3), dtype=np.uint8)
    current_color = (0, 0, 0)

    def show_color(event, x, y, flags, param):
        nonlocal current_color
        if event == cv2.EVENT_MOUSEMOVE:
            if not hasattr(show_color, "ret") or not hasattr(show_color, "frame"):
                return
            if not show_color.ret:
                return
            color = show_color.frame[y, x]
            b, g, r = int(color[0]), int(color[1]), int(color[2])
            current_color = (b, g, r)
            img_copy = show_color.frame.copy()
            cv2.circle(img_copy, (x, y), 5, (255, 255, 255), 2)
            cv2.putText(img_copy, f'R:{r} G:{g} B:{b}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.imshow('Color Identifier', img_copy)

            color_txt_window[:] = (b, g, r)
            text = f'R:{r} G:{g} B:{b}'
            cv2.putText(color_txt_window, text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255) if (r+g+b)<384 else (0,0,0), 2)
            cv2.imshow('Color Text', color_txt_window)

    cv2.namedWindow('Color Identifier')
    cv2.setMouseCallback('Color Identifier', show_color)
    cv2.namedWindow('Color Text')
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        show_color.ret = ret
        show_color.frame = frame
        cv2.imshow('Color Identifier', frame)
      
        color_txt_window[:] = current_color
        text = f'R:{current_color[2]} G:{current_color[1]} B:{current_color[0]}'
        cv2.putText(color_txt_window, text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255) if sum(current_color)<384 else (0,0,0), 2)
        cv2.imshow('Color Text', color_txt_window)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

identify_color_realtime()