from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.tools import wait
from pybricks.robotics import Car
from pybricks.pupdevices import Motor
from dubinsPathPlanning import dubin_calculation
from vehicleConstants import L_car, psi_max, v, max_power, max_angle, r_turn_min
from controlConstants import KP_POSITION, KP_STEERING
from debugFunctions import infoTrayectoria, infoControl
import umath


CM_TO_ANGLE = 22 # 3600 grados / 162cm
ANGLE_TO_CM = 0.045

hub = TechnicHub()
steering = Motor(Port.D, Direction.CLOCKWISE)
front = Motor(Port.B, Direction.CLOCKWISE)
rear = Motor(Port.A, Direction.CLOCKWISE)
audi = Car(steering, [front, rear])

def generateThetaReference(theta, path_type, delta_s, r_turn_min):
    # Inicia el control de posicion y direccion
    # Generación de coordenadas de referencia
    # Es importante notar que la referencia de posición es siempre la misma. delta_s. Se agrega delta_s a la posición actual. 
    # La referencia de orientación se actualiza en cada iteración.
    if path_type == 'R':
        theta = theta + delta_s / r_turn_min
    elif path_type == 'L':
        theta = theta - delta_s / r_turn_min
    else:
        theta = theta
    return theta


def controlDelVehiculo(L, path_type, estimated_driving_time, show_debug_info = False, h = 20, delta_error=1):
    # IMPORTANTE: h en MILISEGUNDOS
    # Inicializar las variables
    resetAllSensors()
    elapsed_time = 0
    position_reference = 0.0
    steering_reference = 0.0
    max_driving_time = estimated_driving_time * 1.2 * 1000.0 # Tiempo máximo de conducción, en milisegundos
    delta_s = h * v / 1000.0 # Distancia recorrida en cada iteración

    for i in range(len(L)):
        for _ in range(int(L[i] / delta_s)):
            # Adquirir el estado actual del vehículo
            #position_vehicle = 0.0
            #steering_vehicle = 0.0
            # TODO: Adquirir los valores reales
            position_vehicle = 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM
            steering_vehicle = hub.imu.heading()
            
            # Evaluar si se llegó al final del recorrido
            if abs(position_vehicle - sum(L)) < delta_error:
                print("Llegó al final del recorrido")
                # TODO: Detener el vehículo
                audi.drive_power(0)
                audi.steer(0)
                hub.light.on(Color.RED)  # Indica que se detuvo
                return

            # Actualizar las referencias
            position_reference += delta_s
            steering_reference = generateThetaReference(
                steering_reference, path_type[i], delta_s, r_turn_min
            )  # devuelve RADIANES

            # Calcular los errores
            position_error = position_reference - position_vehicle
            steering_error = umath.degrees(steering_reference) - steering_vehicle

            # Calcular las acciones de control
            position_command = int(min(max(KP_POSITION * position_error, -max_power), max_power))
            steering_command = int(min(max(KP_STEERING * steering_error, -max_angle), max_angle))

            # TODO: Ejecutar las acciones de control
            audi.steer(steering_command)
            audi.drive_power(position_command)

            wait(h)  # Esperar un intervalo de tiempo
            elapsed_time += h
            if elapsed_time > max_driving_time:
                print("Tiempo máximo de conducción alcanzado")
                # TODO: Detener el vehículo
                audi.drive_power(0)
                audi.steer(0)
                hub.light.on(Color.RED)  # Indica que se detuvo
                wait(2000)
                return

            # Imprimir los valores actuales
            if(elapsed_time % 500 == 0 and show_debug_info):
                infoControl(elapsed_time / 1000.0, position_reference, position_vehicle, position_error, steering_reference, steering_vehicle, steering_error)
            

    # TODO: Detener el vehículo y la dirección después de completar el recorrido
    audi.drive_power(0)
    audi.steer(0)
    hub.light.on(Color.RED)  # Indica que se detuvo
    wait(2000)
    print("Conducción completada en {:.2f} seg.".format(elapsed_time))




def resetAllSensors():
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)

   
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

 
def setup():
    hub.light.on(Color.RED)
    hub.imu.settings(1.5, 250)



if __name__ == "__main__":
    print("Si la batería está baja no sube el programa")
    print(hub.battery.voltage() / 1000, hub.battery.current(),  hub.imu.settings(), hub.imu.ready(), hub.imu.stationary())
    
    setup()
    #testSteeringControl()
    #testEncoders()
    #testSteeringAndPositionControl(-45, 120, True, 1)
    s = [0.0, 0.0, umath.radians(0)]  # Postura inicial
    g = [120.0, 120.0, umath.radians(0)]  # Postura final
    
    L, path_type = dubin_calculation(s, g, r_turn_min)
    # Cómputo de la duración estimada de la trayectoria en función de la velocidad del vehículo. Se agrega un 20%
    estimated_driving_time = sum(L) / v

    infoTrayectoria(s, g, L, path_type, estimated_driving_time)
        
    print("Iniciando control")
    controlDelVehiculo(L, path_type, estimated_driving_time)   
     
    # Asegúrate de limpiar recursos aquí si es necesario
    print("Finalizando el programa correctamente.")
    hub.light.on(Color.GREEN)
    wait(3000)


