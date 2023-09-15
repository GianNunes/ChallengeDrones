from djitellopy import Tello
import cv2

# Dicionários existentes
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

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

# Main loop
while True:
    # Receber o frame atual
    frame = drone.get_frame_read().frame
    
    # Detectar ArUco's no frame
    corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    
    # Se detectado algum ID
    if ids is not None:
        # Declara lista de ids
        ids = ids.flatten()
        
        # Loop dos ArUcos detectados
        for i in range(len(ids)):
            # Printa os ids
            print("Detected ArUco Marker ID:", ids[i])
    
    # Se detectado algum vertice
    if len(corners) > 0:

        # Para cada vertice (assume que existem 4 para cada ID?)
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

            # Desenha o centro
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

			# Desenha o ID ao lado do vertice superior esquerdo
            cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),

                # Fonte
				cv2.FONT_HERSHEY_SIMPLEX,

                # Escala, Cor e Espessura do texto
				0.5, (0, 255, 0), 2)

    # Atualiza o frame da stream
    cv2.imshow("Tello", frame)
    
    # Se apertar 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Sai do loop
        break

# Desliga a stream
drone.streamoff()

# Fecha a janela
cv2.destroyAllWindows()
