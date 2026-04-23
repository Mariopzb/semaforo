import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    red_low1 = np.array([0, 100, 100])
    red_high1 = np.array([10, 255, 255])
    red_low2 = np.array([160, 100, 100])
    red_high2 = np.array([180, 255, 255])
    
    # Verde
    #green_low = np.array([35, 100, 100])
    #green_high = np.array([85, 255, 255])

    green_low = np.array([35, 40, 40]) 
    green_high = np.array([85, 255, 255])
    
    # Amarillo
    yellow_low = np.array([20, 100, 100])
    yellow_high = np.array([32, 255, 255])

    # Crear máscaras
    mask_red = cv2.addWeighted(cv2.inRange(hsv, red_low1, red_high1), 1, cv2.inRange(hsv, red_low2, red_high2), 1, 0)
    mask_green = cv2.inRange(hsv, green_low, green_high)
    mask_yellow = cv2.inRange(hsv, yellow_low, yellow_high)

    # Dibujar contornos o etiquetas simples
    def draw_label(mask, color_name, bgr_color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), bgr_color, 2)
                cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr_color, 2)

    draw_label(mask_red, "ROJO", (0, 0, 255))
    draw_label(mask_green, "VERDE", (0, 255, 0))
    draw_label(mask_yellow, "AMARILLO", (0, 255, 255))

    cv2.imshow('Detector de Colores', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()