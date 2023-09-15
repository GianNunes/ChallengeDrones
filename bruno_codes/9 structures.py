# region: Print script
if __name__ == "__main__":
    tello = initialize()
    arucoDict, arucoParams = initialize_aruco()

    while True:
        frame = tello.get_frame_read().frame

        result = process_image(frame, arucoDict, arucoParams)

        if result is not None:
            id, distance = result
            print(id)
        else:
            print("No ArUco markers detected")

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            break
    tello.streamoff()
#endregion

# region: Constant distance (not 100%)
if __name__ == "__main__":
    tello = initialize()
    arucoDict, arucoParams = initialize_aruco()

    # Take off once at the beginning
    tello.takeoff()

    while True:
        frame = tello.get_frame_read().frame

        result = process_image(frame, arucoDict, arucoParams)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        if result is not None:
            id, distance = result
            print(distance)
            if distance > 2:
                tello.send_rc_control(0, 10, 0, 0)  # Move forward
            elif distance < 2:
                tello.send_rc_control(0, -10, 0, 0)  # Move backward
    land(tello)
#endregion

# region: 4-12 Rotation Loop
if __name__ == "__main__":
    tello = initialize()
    arucoDict, arucoParams = initialize_aruco()

    # Take off once at the beginning
    tello.takeoff()
    tello.send_rc_control(0, 0, 0, 0)

    while True:
        frame = tello.get_frame_read().frame
        
        cv2.imshow('Tello Tracking', frame)
        # Process ArUco markers and store the result
        result = process_image(frame, arucoDict, arucoParams)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            break
        
        if result is not None:
            id, distance = result
            if id == 12:
                rotate_right_90(tello)
                tello.send_rc_control(0, 0, 0, 0)
            elif id == 4:
                rotate_left_90(tello)
                tello.send_rc_control(0, 0, 0, 0)
        else:
            print("No ArUco markers detected")
#endregion

# region: Functioning Distance test (for the calibration)
if __name__ == "__main__":
	tello = initialize()
	arucoDict, arucoParams = initialize_aruco()

	while True:
		frame = tello.get_frame_read().frame

		# Process ArUco markers and store the result
		result = process_image(frame, arucoDict, arucoParams)

		if result is not None:
			id, distance = result
			print(id)
		else:
			print("No ArUco markers detected")

		key = cv2.waitKey(1) & 0xFF
		if key == ord(' '):
			break
#endregion

























