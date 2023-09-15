from djitellopy import Tello
import time
import math
import cv2

list_id = {}
list_distance = {}

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
    return drone  # Return the drone object

class Controller:
    def __init__(self):
        self.tello = Tello()
        self.stack_angle = []
        self.stack_x = []
        self.stack_y =[]


# region Section 1: Primitive Functions
    def move_to(self, x_coordinate, y_coordinate, z_coordinate, speed_value):
        self.tello.go_xyz_speed(x_coordinate, y_coordinate, z_coordinate, speed_value)
        movement_time = max(abs(x_coordinate), abs(y_coordinate), abs(z_coordinate))/speed_value
        self.sleep(movement_time)

    def rotate_right_90(self):
        self.tello.rotate_counter_clockwise(90)
        self.tello.stack_angle.append("-90")
        time.sleep(3)

    def rotate_left_90(self):
        self.tello.rotate_clockwise(90)
        self.tello.stack_angle.append("+90")
        time.sleep(3)

    def perform_one_quarter_curve(self):
        radius = 200
        x1 = radius
        x2 = radius
        y2 = radius
        speed = 5

        estimated_time = int((math.pi / 2) * radius / speed)
        self.tello.curve_xyz_speed(x1, 0, 0, x2, y2, 0, speed*10)

        time.sleep(estimated_time)

    def get_rotation_stack(self):
        return self.rotation_stack
    
    def capture(self):
        while True:
            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
            arucoParams = cv2.aruco.DetectorParameters()

            frame = self.tello.get_frame_read().frame
            time.sleep(1)
            corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
            # Se detectados
            if ids is not None:
                for (marker_corner, marker_id) in zip(corners, ids):
                    # Converte marker_id para um tuple
                    marker_id_tuple = tuple(marker_id)
                    
                    # Verifica se o marcador ainda não foi detectado
                    if marker_id_tuple not in list_id:
                        # Calcula a distância
                        corners = marker_corner.reshape((4, 2))
                        (top_left, top_right, bottom_right, bottom_left) = corners

                        top_right = (int(top_right[0]), int(top_right[1]))
                        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                        top_left = (int(top_left[0]), int(top_left[1]))

                        marker_size = 100/1.125  # Tamanho ajustado na tela (50 mm)
                        distance = marker_size / (((top_right[0] - top_left[0]) ** 2 + (top_right[1] - top_left[1]) ** 2) ** 0.5)

                        # Salva o ID
                        list_id[marker_id_tuple] = marker_id
                        # Salva a distância
                        list_distance[marker_id_tuple] = distance
            cv2.imshow("Tello", frame)
            print(list_id)
            if cv2.waitKey(1) & 0xFF == ord(' '):
                drone.land()
# endregion

# region Section 2: Structured Functions


# endregion


if __name__ == "__main__":
    
    controller = Controller()  # Rename the instance to 'controller'
    drone = initialize()  # Initialize the drone and get the drone object
    drone.takeoff()
    time.sleep(1)
    drone.go_xyz_speed(250, 0, 0, 20)
    time.sleep(4)
    drone.rotate_clockwise(90)
    time.sleep(4)
    drone.go_xyz_speed(100, 0, 0, 20)
    time.sleep(4)
    drone.land()
    
