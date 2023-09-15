from djitellopy import Tello
import time

# Inicializa o drone Tello
drone = Tello()

# Conecta-se ao drone
drone.connect()

# Inicia a transmissão de vídeo (você pode comentar esta linha se não precisar da transmissão de vídeo)
drone.streamon()

# Variáveis
(x, y) = (100, 100)

# Decola
drone.takeoff()
time.sleep(1)

# Vai até o início da prova
drone.go_xyz_speed(x, y, 100, 60)
time.sleep(3)

# Rotaciona o drone 90 graus para direita
drone.rotate_clockwise(90)
time.sleep(3)

# Pousa o drone
drone.land()
time.sleep(1)

# Interrompe a transmissão de vídeo
drone.streamoff()
