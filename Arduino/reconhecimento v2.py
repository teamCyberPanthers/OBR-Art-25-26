import cv2
import numpy as np
import serial.tools.list_ports
import time

# ==== CONFIGURAÇÃO SERIAL ====
ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portsList = []

print("Portas disponíveis:")
for one in ports:
    portsList.append(str(one))
    print(str(one))

com = input("Digite o número da COM (ex: 3 para COM3): ").strip()

use = None
for port in portsList:
    if port.startswith("COM" + com):
        use = "COM" + com
        print("Usando:", use)
        break

if use is None:
    print(f"Porta COM{com} não encontrada!")
    exit()

serialInst.baudrate = 9600
serialInst.port = use
serialInst.open()

# ==== WEBCAM E TRACKBARS ====
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Erro ao abrir webcam")
    exit()

cv2.namedWindow('Trackbars')

def nothing(x):
    pass

# Criar sliders HSV
cv2.createTrackbar('H Lower', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('H Upper', 'Trackbars', 20, 179, nothing)
cv2.createTrackbar('S Lower', 'Trackbars', 30, 255, nothing)
cv2.createTrackbar('S Upper', 'Trackbars', 150, 255, nothing)
cv2.createTrackbar('V Lower', 'Trackbars', 60, 255, nothing)
cv2.createTrackbar('V Upper', 'Trackbars', 255, 255, nothing)

ultimo_envio = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Falha ao capturar o frame da webcam")
        break

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Leitura dos sliders
    hL = cv2.getTrackbarPos('H Lower', 'Trackbars')
    hU = cv2.getTrackbarPos('H Upper', 'Trackbars')
    sL = cv2.getTrackbarPos('S Lower', 'Trackbars')
    sU = cv2.getTrackbarPos('S Upper', 'Trackbars')
    vL = cv2.getTrackbarPos('V Lower', 'Trackbars')
    vU = cv2.getTrackbarPos('V Upper', 'Trackbars')

    lower = np.array([hL, sL, vL])
    upper = np.array([hU, sU, vU])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Detectar a quantidade de área branca na máscara
    area = cv2.countNonZero(mask)
    cv2.putText(frame, f"Area: {area}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    if area > 10000 and time.time() - ultimo_envio > 2:
        print("Objeto detectado - enviando comando '1'")
        serialInst.write(b'1')
        ultimo_envio = time.time()

    cv2.imshow('Frame', frame)
    cv2.imshow('Mask', mask)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC para sair
        break

cap.release()
cv2.destroyAllWindows()
serialInst.close()
