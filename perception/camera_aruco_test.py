import cv2
from djitellopy import Tello
import time
import numpy as np
import math
from enum import Enum


def main():
	drone = Tello()
	drone.connect()
	drone.streamon()
	drone.takeoff()
	# time.sleep(10)
	f = cv2.FileStorage('calibration.xml', cv2.FILE_STORAGE_READ)
	cameraMatrix = f.getNode("intrinsic").mat()
	distCoeffs = f.getNode('distortion').mat()
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
	parameters = cv2.aruco.DetectorParameters()

	intrinsic = cameraMatrix
	distortion = distCoeffs

	state = 0
	
    
	while(True):
		frame =  drone.get_frame_read().frame
		imageSize = (frame.shape[0], frame.shape[1])
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
		try:
			if len(markerCorners) > 0:
				frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

				rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion)
				#frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec, tvec, 0.1)

				#text = 'x = ' + str(tvec[0][0][0]) + 'y = ' + str(tvec[0][0][1]) + 'z = ' + str(tvec[0][0][2])

				cv2.putText(frame, str(tvec), (10, 40), cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 255), 1, cv2.LINE_AA)
				cv2.putText(frame, str(rvec), (10, 400), cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 255), 1, cv2.LINE_AA)

				key = cv2.waitKey(33)


				if key != -1:
					# drone.keyboard(key)
					cv2.destroyAllWindows()
				else:
					id = markerIds[0][0]

					# if id == 2:
						# drone.flip('f')       
			
					##tvec is (LR, UPDOWN, distance)
					up_down = tvec[0][0][1] - 5
					if up_down > 10:
						print("up >10")
						# drone.move_down(up_down/100)
					elif up_down < -10:
						print("up <10")
						# drone.move_up(-up_down/100)

					# erro = atual - setpoint
					
					distance = tvec[0][0][2] - 1200
					# print('erro =' distance)
					if distance > 0:
						print("erro = >0", distance)
						if(distance>300):
							#drone.move_forward(max(min(int(distance/100),200),200))
							drone.go_xyz_speed(max(min(int(distance/100),20),20),0,0,20) # Move o drone até o inicio
							drone.send_rc_control(0,max(min(int(distance/100),20),20),0,0)
						else:
							drone.send_rc_control(0,0,0,0)
						# drone.move_forward()

					else:
						print("erro = >0", distance)
						if(distance<300):
							#drone.move_back(min(max(int(-distance/100),-200),-200))
							#drone.go_xyz_speed(min(max(int(-distance/100),-20),-20),0,0,20) # Move o drone até o inicio
							drone.send_rc_control(0,min(max(int(-distance/100),-20),-20),0,0)
						else:
							drone.send_rc_control(0,0,0,0)


					left_right = tvec[0][0][0]
					if left_right > 10:
						print("move right > 0 ")
						# drone.move_right(left_right/100)
					elif left_right < -10:
						print("move right < 0 ")
						# drone.move_left(-left_right/100)


					dst, jaco = cv2.Rodrigues(rvec[0][0])
					z_ = np.array([dst[0][2], dst[1][2], dst[2][2]])
					v = np.array([z_[0], 0, z_[2]])
					
					degree = math.atan2(z_[2], z_[0])
					degree = -degree * 180  / math.pi
					cv2.putText(frame, str(degree), (10, 200), cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 255), 1, cv2.LINE_AA)

					if degree > 100:
						print("rotatet > 0 ")

						# drone.rotate_cw(10)
					elif degree < 80:
						print("rotatet < 0 ")

						# drone.rotate_ccw(10)
			else:
				try:
					#drone.rotate_clockwise(45)
					drone.send_rc_control(0,0,0,0)
				except:
					pass
		except:
			pass
		# cv2.imshow('frame', frame)

		key = cv2.waitKey(33)
		if key != -1:
			# drone.keyboard(key)
			cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
