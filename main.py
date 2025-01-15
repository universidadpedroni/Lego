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
    print("Conducción completada en {:.2f} seg.".format(elapsed_time / 1000.0))



def resetAllSensors():
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)

   

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
    s = [0.0, 0.0, umath.radians(0.0)]  # Postura inicial
    g = [120.0, 120.0, umath.radians(90.0)]  # Postura final
    
    L, path_type = dubin_calculation(s, g, r_turn_min)


    estimated_driving_time = sum(L) / v
    infoTrayectoria(s, g, L, path_type, estimated_driving_time)
    print("Iniciando control trayectoria")
    controlDelVehiculo(L, path_type, estimated_driving_time)
    #PATH = [(0.0, 0.0, 0.0), (0.0, 0.0, 90.0), (0.0, 0.0, 180.0), (0.0, 0.0, 270.0), (0.0, 0.0, 0.0)]
    #PATH = [(0.0, 0.0, umath.radians(0.0)), (120.0, 0.0, umath.radians(0.0))]

    '''
    for i in range(len(PATH) - 1):
        L, path_type = dubin_calculation([PATH[i][0], PATH[i][1], umath.radians(PATH[i][2])], 
                                         [PATH[i+1][0], PATH[i+1][1], umath.radians(PATH[i+1][2])], r_turn_min)
        # Cómputo de la duración estimada de la trayectoria en función de la velocidad del vehículo. Se agrega un 20%
        estimated_driving_time = sum(L) / v
        infoTrayectoria(PATH[i], PATH[i+1], L, path_type, estimated_driving_time)
        print("Iniciando control trayectoria {}".format(i+1))
        controlDelVehiculo(L, path_type, estimated_driving_time, True)
        wait(2000)
    ''' 
    # Asegúrate de limpiar recursos aquí si es necesario
    print("Finalizando el programa correctamente.")
    hub.light.on(Color.GREEN)
    wait(3000)


