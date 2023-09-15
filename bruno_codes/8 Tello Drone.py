import cv2
import threading
import time
from djitellopy import Tello
from simple_pid import PID
import paho.mqtt.client as mqtt

broker = ''
port = 1883
topic = "IDAtual"
ProximoID = 1

def initialize():
    drone = Tello()
    drone.connect()
    drone.streamon()
    return drone

#region: global variables
stack_angle = []

group_a = {2,3,4,5}
group_b = {6,7,8,9}
group_c = {10,11,12,13}
group_d = {14,15,16,17}
group_e = {18,19,20,21}
group_p = {22,23,24}

id_to_group = {}

for identified in group_a:
    id_to_group[identified] = 'group_a'
for identified in group_b:
    id_to_group[identified] = 'group_b'
for identified in group_c:
    id_to_group[identified] = 'group_c'
for identified in group_d:
    id_to_group[identified] = 'group_d'
for identified in group_e:
    id_to_group[identified] = 'group_e'
for identified in group_p:
    id_to_group[identified] = 'group_p'
#endregion

def get_rotation_roll_back():
    return sum(stack_angle)

def rotate_right(angle):
    tello.rotate_counter_clockwise(-1*angle)
    stack_angle.append(-1*angle)
    time.sleep(1)

def rotate_left(angle):
    tello.rotate_counter_clockwise(angle)
    stack_angle.append(angle)
    time.sleep(1)

def initialize_aruco():
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    arucoParams = cv2.aruco.DetectorParameters()
    return arucoDict, arucoParams

def process_image(frame, arucoDict, arucoParams):
    corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

    if ids is not None:
        if corners is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for (marker_corner, marker_id) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                marker_size = 10250  # Tamanho do marker_area a 1 metro de distância
                marker_area = cv2.contourArea(marker_corner)
                distance = (marker_size / marker_area) ** 0.5
                height, width, _ = frame.shape

                aruco_x = int((topLeft[0] + bottomRight[0]) / 2.0)
                frame_x = width // 2

                return ids, distance, aruco_x, frame_x



class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        p = self.kp * error

        self.integral += error
        i = self.ki * self.integral

        d = self.kd * (error - self.prev_error)
        self.prev_error = error

        control_output = p + i + d

        return control_output

def align(D, center_x, aruco_x):
    kp_d = 10
    ki_d = 0.1
    kd_d = 0.01

    kp_x = 1
    ki_x = 0.1
    kd_x = 0.01

    pid_x = PIDController(kp_x, ki_x, kd_x)
    error_x = center_x - aruco_x
    rc_x = pid_x.update(error_x)

    desired_distance = 2.0

    pid_distance = PIDController(kp_d, ki_d, kd_d)
    error_distance = desired_distance - D
    rc_distance = pid_distance.update(error_distance)

    rc_x = max(min(rc_x, 20), -20)
    rc_distance = max(min(rc_distance, 20), -20)
    rc_x = -1 * rc_x
    return rc_x, rc_distance

def begin(id):
    tello.takeoff()
    x=0
    y=0
    z=0
    v=60
    tello.go_xyz_speed(x,y,z,v)
    time.sleep(5)
    tello.rotate_clockwise(90)
    if id == 1:
        IDAtual = send(1)
        ProximoID = receive()
    if ProximoID != 1:
        tello.move_up(150)
        
def go_around(tello):
    tello=Tello()
    tello.go_xyz_speed(200, 310, 0, 50)
    time.sleep(4)
    tello.rotate_clockwise(90)
    time.sleep(4)

def send(id):

    return

def receive():
    IDAtual = 12
    ProximoID = 4

def frame_thread():
    global frame
    while True:
        try:
            frame = tello.get_frame_read().frame
        except Exception:
            print ('\nExit . . .\n')
            break

if __name__ == "__main__":

    tello = initialize()


    #frame = None
    #frameThread = threading.Thread(target=frame_thread)
    #frameThread.daemon = True
    #frameThread.start()
    arucoDict, arucoParams = initialize_aruco()
    frame = tello.get_frame_read().frame

    time.sleep(8)

    #tello.takeoff()
    #tello.send_rc_control(0,0,0,0)

    while True:

        result = process_image(frame, arucoDict, arucoParams)
        if result is not None and all(item is not None for item in result):
            ID, distance, aruco_x, frame_x = result
            print(result)
            searched = 5
            searched_group = id_to_group.get(searched)

            if identified in id_to_group and id_to_group.get(identified) == searched_group:
                (rc_x, rc_d) = align(distance, frame_x, aruco_x)
                #tello.send_rc_control(int(rc_x), int(rc_d), 0, 0)


                if distance<1:
                    #tello.send_rc_control(int(rc_x), int(rc_d), 0, 0)
                    print(f"Distance: {distance}, RC_d: {rc_d}, RC_x: {rc_x}")


                if distance<1:
                    #tello.send_rc_control(int(rc_x), int(rc_d), 0, 0)
                    while searched != identified:
                        frame = tello.get_frame_read().frame
                        result = process_image(frame, arucoDict, arucoParams)


                        if result is not None:
                            identified = int(result[0]) 
                            #go_around(tello)
                            print('Around')
                        else:
                            #tello.send_rc_control(0,0,0,0)
                            print('No markers detected.')


        else:
            #tello.send_rc_control(0,0,0,0)
            print('No markers detected.')