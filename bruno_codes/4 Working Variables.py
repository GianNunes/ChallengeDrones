from djitellopy import Tello
import cv2
import time

data_id = {}
data_distance = {}



(xi, yi) = (25, 25)

# Função para inicializar o drone e o streaming
def initialize():
    # Inicializar o drone
    drone = Tello()
    # Conectar com o drone
    drone.connect()
    # Inicializar o streaming
    drone.streamon()
    # Abrir janela de streaming
    cv2.namedWindow("Tello")
    # Definir um dicionário ArUco
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    # Parâmetros do OpenCV
    arucoParams = cv2.aruco.DetectorParameters()
    # Coordenadas para o início da competição
  
    return drone, arucoDict, arucoParams

# Vai até o início da prova
def begin(drone, x, y):
    # Move para (x, y, 1) com uma velocidade de 90 cm/s
    x_coordinate=x
    y_coordinate=y
    drone.takeoff()
    drone.go_xyz_speed(x_coordinate, y_coordinate, 100, 60)
    drone.rotate_counter_clockwise(90)  # Rotate 90 degrees to the left
    drone.land()
    # Tempo de execução
    tf = ((x_coordinate ** 2 + y_coordinate ** 2) ** 0.5) / (90)

    # Aguarda a execução
    time.sleep(tf)

# Função para detecção dos ids
def frame_receipt_loop(drone, arucoDict, arucoParams):
    while True:
        # Recebe os frames
        frame = drone.get_frame_read().frame

        # Detecta marcadores ArUco nos frames
        corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        
        # Se detectados
        if ids is not None:
            for (marker_corner, marker_id) in zip(corners, ids):
                # Converte marker_id para um tuple
                marker_id_tuple = tuple(marker_id)
                
                # Verifica se o marcador ainda não foi detectado
                if marker_id_tuple not in data_id:
                    # Calcula a distância
                    corners = marker_corner.reshape((4, 2))
                    (top_left, top_right, bottom_right, bottom_left) = corners

                    top_right = (int(top_right[0]), int(top_right[1]))
                    bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                    bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                    top_left = (int(top_left[0]), int(top_left[1]))

                    marker_size = 50  # Tamanho ajustado na tela (50 mm)
                    distance = marker_size / (((top_right[0] - top_left[0]) ** 2 + (top_right[1] - top_left[1]) ** 2) ** 0.5)

                    # Salva o ID
                    data_id[marker_id_tuple] = marker_id
                    # Salva a distância
                    data_distance[marker_id_tuple] = distance
        
        cv2.imshow("Tello", frame)
        print(data_id)
        print(data_distance)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            drone.land()
            break

if __name__ == "__main__":
    initialize
    drone, arucoDict, arucoParams = initialize()
    #begin(drone, x, y)
    frame_receipt_loop(drone, arucoDict, arucoParams)  # Move this function call here
    drone.streamoff()