import cv2
import cv2.aruco as aruco
from djitellopy import Tello
from simple_pid import PID
import time
import math

# Initialize a Tello drone object
tello = Tello()

# Connect to the drone
tello.connect()

# Start video streaming
tello.streamon()

# Fly
tello.takeoff()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters()

# Initialize PID controllers for x and y axis
pid_x = PID(0.1, 0.01, 0.05, setpoint=0)
pid_y = PID(0.1, 0.01, 0.05, setpoint=0)

while True:
    # Get the current frame from Tello
    frame_read = tello.get_frame_read()
    frame = frame_read.frame

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    threshold_positive = 100
    threshold_negative = -100

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    
    if ids is not None:
        # Draw the detected marker corners on the image
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Assuming marker with ID 0 is our reference, find it's index in ids list
        index = list(ids.flatten()).index(1)

        # Get the center of the ArUco marker
        center = tuple(map(int, corners[index][0].mean(axis=0)))

        # Calculate error between center of frame and center of marker
        error_x = frame.shape[1]/2 - center[0]
        error_y = frame.shape[0]/2 - center[1]

        # Get adjustment for drone using PID controllers
        adjustment_x = pid_x(error_x)
        adjustment_y = pid_y(error_y)

        # Only send control command if error is above threshold
        if error_x > threshold_positive:
            tello.send_rc_control(-int(adjustment_x), 0, 0, 0)
        elif error_x < threshold_negative:
            tello.send_rc_control(int(adjustment_x), 0, 0, 0)

        if error_y > threshold_positive:
            tello.send_rc_control(0, 0, int(adjustment_y), 0)
        elif error_y < threshold_negative:
            tello.send_rc_control(0, 0, -int(adjustment_y), 0)
        # Display the resulting frame
        cv2.imshow('Tello Tracking', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release cap and destroy all windows
cv2.destroyAllWindows()
