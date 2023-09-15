if __name__ == "__main__":
    tello = initialize()
    arucoDict, arucoParams = initialize_aruco()

@   begin(tello, x, y)

    while True:
        frame = tello.get_frame_read().frame
        cv2.imshow('Tello Tracking', frame)
        result = process_image(frame, arucoDict, arucoParams)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            break
        
        if result is not None:
            id, distance = result
                

                if ProximoID is in group_b
                    if id is in group_b:
                        (rc_x, rc_y) = align(tello, frame, id)
                        tello.send_rc_control(rc_x, rc_y, 0, 0)
                        if ProximoID == id:
                            send_to_abii
                        else go_around

                if ProximoID is in group_c
                    if id is in group_c:
                        (rc_x, rc_y) = align(tello, frame, id)
                        tello.send_rc_control(rc_x, rc_y, 0, 0)
                        if ProximoID == id:
                            send_to_abii
                        else go_around

                if ProximoID is in group_d
                    if id is in group_d:
                        (rc_x, rc_y) = align(tello, frame, id)
                        tello.send_rc_control(rc_x, rc_y, 0, 0)
                        if ProximoID == id:
                            send_to_abii
                        else go_around

                if ProximoID is in group_e
                    if id is in group_e:
                        (rc_x, rc_y) = align(tello, frame, id)
                        tello.send_rc_control(rc_x, rc_y, 0, 0)
                        if ProximoID == id:
                            send_to_abii
                        else go_around

                if ProximoID is in group_p
                    if id is in group_p:
                        (rc_x, rc_y) = align(tello, frame, id)
                        tello.send_rc_control(rc_x, rc_y, 0, 0)
                        if ProximoID == id:
                            if distance < 10
                                tello.land
@                           else get_close(id)

        if keyboard.is_pressed( ):
            break
            
tello.land

if __name__ == "__main__":

tello = initialize()
arucoDict, arucoParams = initialize_aruco()

frame = None
frameThread = threading.Thread(target=frame_thread)
frameThread.daemon = True
frameThread.start()

time.sleep(4)

tello.takeoff() # Levanta voo
tello.send_rc_control(0,0,0,0) # Estabiliza
while True:
    
    result = process_image(frame, arucoDict, arucoParams) # Extrai a distância e a coordenada x (do id em relação ao centro do frame)
    if result is not None:
        ID, distance, xc, xa = result # Separa as variáveis
        identified = int(result[0])
        searched = 5 # Id desejado
        searched_group = id_to_group.get(searched) # Define o grupo desejado

        if identified in id_to_group and id_to_group.get(identified) == searched_group: # Se o Id do frame e o Id desejado são do mesmo grupo
            (rc_x, rc_d) = align(distance, xc, xa) # Itera a velocidade x e y até alinhar e se posicionar a 2 metros do Id
            tello.send_rc_control(int(rc_x), int(rc_d), 0, 0) # Move o drone
            print(f"Distance: {distance}, RC_d: {rc_d}, RC_x: {rc_x}")

            if distance<2:
                while searched != identified: # Enquanto o Id não é o desejado
                    result = process_image(frame, arucoDict, arucoParams)
                    if result is not None:
                        identified = int(result[0]) 
                        go_around(tello) # Roda em torno do Totem
                        print('Around')
                    else:
                        tello.send_rc_control(0,0,0,0)
                        print('No markers detected.')


    else:
        tello.send_rc_control(0,0,0,0)
        print('No markers detected.')