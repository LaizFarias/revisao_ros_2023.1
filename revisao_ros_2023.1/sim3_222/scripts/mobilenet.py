import numpy as np
import cv2


CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"]


net = cv2.dnn.readNetFromCaffe('mobilenet_detection/MobileNetSSD_deploy.prototxt.txt','mobilenet_detection/MobileNetSSD_deploy.caffemodel')
   
CONFIDENCE = 0.2
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))


def detect(net, frame, CONFIDENCE, COLORS, CLASSES):
    """
        Recebe:
            net - a rede carregada
            frame - uma imagem colorida BGR
            CONFIDENCE - o grau de confiabilidade mínima da detecção
            COLORS - as cores atribídas a cada classe
            CLASSES - o array de classes
        Devolve: 
            img - a imagem com os objetos encontrados
            resultados - os resultados da detecção no formato
             [(label, score, point0, point1),...]
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()
    
    results = []

    for i in np.arange(0, detections.shape[2]):

        confidence = detections[0, 0, i, 2]

        if confidence > CONFIDENCE:
        
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            
            y = startY - 15 if startY - 15 > 15 else startY + 15

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    return image, results
