from djitellopy import tello
import cv2
import cv2.aruco as aruco

drone = tello.Tello()
cap = cv2.VideoCapture(0)
drone.connect()
drone.streamon()

while True:
    img = drone.get_frame_read().frame
    img = cv2.resize(img, (480, 340))
    cv2.imshow("Tello", img)
    cv2.waitKey(1)