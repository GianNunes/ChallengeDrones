from djitellopy import Tello
import cv2
import math
import time
import numpy as np

drone_height=0.22
aruco_height=0.09

# Inizializar o drone
drone = Tello()
# Conectar com o drone
drone.connect()
# Inicializar streaming
drone.streamon()
# Definir um dicionário
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# Definir os parâmetros
arucoParams = cv2.aruco.DetectorParameters()
# Abrir janela de streaming
cv2.namedWindow("Tello")

while True:
    # Receber o frame atual
    frame = drone.get_frame_read().frame

    # Detectar ArUco's no frame
    corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

# region Section 1: Detecção dos IDs
    if ids is not None:
        # Declara lista de ids
        ids = ids.flatten()
        
        # Loop dos ArUcos detectados
        for i in range(len(ids)):
            # Imprime os ids
            print("Detected ArUco Marker ID:", ids[i])

        for (markerCorner, markerID) in zip(corners, ids):

            # Declara os vertices
            corners = markerCorner.reshape((4, 2))

            # Quatro vertices por ArUco
            (topLeft, topRight, bottomRight, bottomLeft) = corners

			# Converte as coordenadas (x,y) de cada vertice em int's
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Cancula o centro do ArUco baseado nos vertices
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            
            # Recebe o tamanho do frame
            height, width, _ = frame.shape

            # coordenadas do centro do frame
            center_x = width // 2
            center_y = height // 2
# endregion

            # Estimar a distância do ArUco baseado no seu tamanho
            marker_size = 65/1.125  # Tamanho ajustado na tela do celular 50 mm dividio por um fator de correção
           
            distance = marker_size / (((topRight[0] - topLeft[0])**2 + (topRight[1] - topLeft[1])**2)**0.5)
            print("Distance in meters:", distance)

            real_size = 65 # Centimetros
            cateto_oposto = real_size / ((center_x-cX)**2+(center_y-cY)**2)**0.5

# region Section 3: Visualização
            # Desenha o centro
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            cv2.circle(frame, (bottomRight[0], bottomRight[1]), 3, (0, 255, 0), -1)
            cv2.circle(frame, (bottomLeft[0], bottomLeft[1]), 3, (0, 255, 0), -1)
            cv2.circle(frame, (topLeft[0], topLeft[1]), 3, (0, 255, 0), -1)
            cv2.circle(frame, (topRight[0], topRight[1]), 3, (0, 255, 0), -1)

            # Desenha o centro
            cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), -1)

			# Desenha o ID ao lado do vertice superior esquerdo
            cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
# endregion

    # Atualiza o frame da stream
    cv2.imshow("Tello", frame)
    
    # Se apertar 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Sai do loop
        break
