from djitellopy import Tello
import cv2
import numpy as np

# Dicionários existentes

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
#drone.takeoff()
# Main loop
cv2.namedWindow("Tello")
while True:
	# Receber o frame atual
	frame = drone.get_frame_read().frame
	cv2.imshow("Tello", frame)
	# Detectar ArUco's no frame
	corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

	# Se detectado algum ID
	if ids is not None:
		# Declara lista de ids
		ids = ids.flatten()
		
		# Loop dos ArUcos detectados
		for i in range(len(ids)):
			# Imprime os ids
			print("Detected ArUco Marker ID:", ids[i])
			
		# Para cada 
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
			
			# Estimar a distância do ArUco baseado no seu tamanho
			marker_size = 10250  # Size of marker_area one meter away
			marker_area = cv2.contourArea(markerCorner) # Size of marker in the frame
			distance = (marker_size / marker_area) ** 0.5
			print("Distance in meters:", distance)

			# Desenha o centro
			cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
			cv2.circle(frame, (bottomRight[0], bottomRight[1]), 3, (0, 255, 0), -1)
			cv2.circle(frame, (bottomLeft[0], bottomLeft[1]), 3, (0, 255, 0), -1)
			cv2.circle(frame, (topLeft[0], topLeft[1]), 3, (0, 255, 0), -1)
			cv2.circle(frame, (topRight[0], topRight[1]), 3, (0, 255, 0), -1)

			# Desenha o ID ao lado do vertice superior esquerdo
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),

				# Fonte
				cv2.FONT_HERSHEY_SIMPLEX,

				# Escala, Cor e Espessura do texto
				0.5, (0, 255, 0), 2)


	# Se apertar 'q'
	if cv2.waitKey(1) & 0xFF == ord('q'):
		# Sai do loop
		break
