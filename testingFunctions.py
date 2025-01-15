def testEncoders(cm:int = 100 ):
    resetAllSensors()
    front.run_target(1000, cm * CM_TO_ANGLE , then=Stop.HOLD, wait=False)
    rear.run_target(1000, cm * CM_TO_ANGLE, then=Stop.HOLD, wait=False)

    testSteeringControl(8000, 80, False, False)
    print(f"Enc Front: {front.angle() * ANGLE_TO_CM} cm, Enc rear: {rear.angle() * ANGLE_TO_CM} cm")


def testSteeringAndPositionControl(ref_steering_angle = 0.0, ref_position = 200, reset_all_sensors = True, delta_pos = 1, show_debug_info = False):
    hub.light.on(Color.GREEN)
    if(reset_all_sensors):
        resetAllSensors()
    
    
    max_steering = 100  # Máximo porcentaje de giro
    max_power = 100     # Máximo porcentaje de potencia
    interval = 5  # Intervalo de espera en milisegundos
    elapsed_time = 0 # Tiempo transcurrido
    max_driving_time = 10000 # Tiempo máximo de conducción
    
    # Inicia el control de dirección
    keep_driving = True
    while keep_driving:
        # Adquisición del estado del vehículo
        enc = 0.5*(front.angle() + rear.angle()) * ANGLE_TO_CM
        steering_angle = hub.imu.heading()
        # Cálculo del error de dirección y posición
        steering_error = ref_steering_angle - steering_angle
        position_error = ref_position - enc

        # Calcula la dirección con límites
        steering_command = int(min(max( 10 * steering_error, -max_steering), max_steering))
        
        
        # Calcula la posición con límites
        position_command = int(min(max( 10 * position_error, -max_power), max_power))
        audi.steer(steering_command)
        audi.drive_power(position_command)

        if(abs(position_error) < delta_pos):
            keep_driving = False
        # Espera y actualiza el tiempo transcurrido
        wait(interval)
        elapsed_time += interval
        if(elapsed_time > max_driving_time):
            keep_driving = False
            print("Tiempo máximo de conducción alcanzado")
        if( elapsed_time % 1000  == 0 and show_debug_info):
            # Imprime los valores actuales
            print(f"Elapsed time: {elapsed_time}", end = " ")
            print(f"Ref: {ref_steering_angle:.2f}, Angle: {steering_angle:.2f}, Error: {steering_error:.2f}, Steering: {steering_command}", end = " ")
            print(f"Ref: {ref_position:.2f}, Enc: {enc:.2f}, Error: {position_error:.2f}, Power: {position_command}")
                 
        

    # Detiene el vehículo y la dirección después de completar el recorrido
    audi.drive_power(0)
    audi.steer(0)
    hub.light.on(Color.RED)  # Indica que se detuvo
    wait(2000)
    print(f"Tiempo de conducción completado. Pulsos de encoder:{enc}" )
    

def testSteeringControl(drive_time=1000, power=80, reset_all_sensors = True, drive = True):
    hub.light.on(Color.GREEN)
    if(reset_all_sensors):
        resetAllSensors()
    
    ref_steering_angle = 0.0  # En degrees
    max_steering = 100  # Máximo porcentaje de giro
    elapsed_time = 0  # Tiempo transcurrido
    interval = 5  # Intervalo de espera en milisegundos
    if(drive):
        audi.drive_power(power)
    # Inicia el control de dirección
    while elapsed_time < drive_time:
        enc = 0.5*(front.angle() + rear.angle())
        steering_angle = hub.imu.heading()
        steering_error = ref_steering_angle - steering_angle

        # Calcula la dirección con límites
        steering_command = int(10 * min(max(steering_error, -max_steering), max_steering))
        audi.steer(steering_command)

        # Imprime los valores actuales
        #print(f"Ref: {ref_steering_angle:.2f}, Angle: {steering_angle:.2f}, Error: {steering_error:.2f}, Steering: {steering_command}")

        # Espera y actualiza el tiempo transcurrido
        wait(interval)
        elapsed_time += interval

    # Detiene el vehículo y la dirección después de completar el tiempo
    audi.drive_power(0)
    
    audi.steer(0)
    hub.light.on(Color.RED)  # Indica que se detuvo
    print(f"Tiempo de conducción completado. Pulsos de encoder:{enc}" )

 