# importando as biblioteca
from ultralytics import YOLO
import cv2 as cv

# Carrengado o modelo treinado
model = YOLO('best.pt')

# capturado o video
capture = cv.VideoCapture(0)

# loop para capturar o video
while True:
    # lendo o video
    _, frame = capture.read()
    # fazendo a predição
    result = model.predict(frame, conf=0.6)
    # mostrando o resultado
    cv.imshow('frame', result[0].plot())
    # verificando se a tecla q foi pressionada
    if cv.waitKey(1) == ord('q'):
        break
# liberando a memoria
capture.release()
# fechando a janela
cv.destroyAllWindows()

