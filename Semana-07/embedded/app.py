#import libraries
import cv2
import requests
import numpy as np
import json
import base64

cap = cv2.VideoCapture(0)  # 0 geralmente se refere à webcam principal

while True:
    ret, frame = cap.read()  # Captura um único quadro de vídeo
    if not ret:
        break  # Se não conseguimos capturar um quadro, terminamos o loop

    # Aqui, você pode converter o quadro em um formato que pode ser enviado via POST.
    # Um jeito comum de fazer isso é codificar o quadro como JPEG e depois codificar o JPEG em base64.
    ret, jpeg = cv2.imencode('.jpg', frame)
    if not ret:
        break

    jpeg_bytes = jpeg.tobytes()
    jpeg_b64 = base64.b64encode(jpeg_bytes).decode('utf-8')

    # Agora, enviamos o quadro para o servidor
    requests.post('http://localhost:8000/upload_image', files={'image': ('image.jpg', jpeg_bytes, 'image/jpeg')})

