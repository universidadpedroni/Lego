# vehicle.py
#from pybricks.robotics import Car
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
#from pybricks.robotics import Car


class Vehicle:
    def __init__(self):
        # Inicializamos los motores con los puertos correspondientes
        self.steering_motor = Motor(Port.D, Direction.CLOCKWISE)  # Gears: 20 20 12 8 
        self.front_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.rear_motor = Motor(Port.A, Direction.CLOCKWISE)
        
        
        
    def setup(self):
        """ Inicializa los motores y devuelve los objetos de los motores """
        print("Inicializando motores...")
        # Aquí podrías agregar lógica extra si fuera necesario en el futuro.
        print("Motores Inicializados")
        return self.rear_motor, self.front_motor, self.steering_motor

    def calibrate_steering_motor(self):
        """ Calibra el motor de dirección (encontrar límites izquierdo y derecho) """
        # Inicializamos intentando llevar el motor al centro
        # Parámetros de calibración (De modificarse, usar calibrate_steering_motor)
        
        self.left_end = self.steering_motor.run_until_stalled(-200, duty_limit=30)
        self.right_end = self.steering_motor.run_until_stalled(200, duty_limit=30)
        
        self.steering_motor.reset_angle((self.right_end - self.left_end) / 2)
        self.steering_motor.run_target(300, 0)
        wait(1000)
        
        print(f"Límite izquierdo: {self.left_end}°")
        print(f"Límite derecho: {self.right_end}°")

        

    def get_motor_status(self):
        """ Devuelve el estado de los motores """
        return {
            'rear_motor': self.rear_motor.angle(),
            'front_motor': self.front_motor.angle(),
            'steering_motor': self.steering_motor.angle()
        }
    
    def reset_motor_status(self):
        """ Resetea el estado de los motores """
        self.rear_motor.reset_angle(True)
        self.front_motor.reset_angle(True)
        print(self.get_motor_status())
    
    def move_with_steering(self, direction, duration):
        """
        Mueve el vehículo con una velocidad específica en el motor de dirección y los motores trasero y delantero.
        :param steering_speed: Velocidad del motor de dirección (negativo para giro a la izquierda, positivo a la derecha)
        :param rear_front_speed: Velocidad de los motores trasero y delantero (positiva para adelante, negativa para atrás)
        :param duration: Tiempo en milisegundos que el vehículo se moverá
        """

        if(direction == 'R'):
            self.steering_motor.run_target(1000, self.right_end,then=Stop.HOLD, wait= False)
        elif(direction == 'L'):
            self.steering_motor.run_target(1000, self.left_end, then=Stop.HOLD ,wait= False)
        elif(direction == 'S'):
            self.steering_motor.run_target(1000, 0, then=Stop.HOLD, wait= False)
        else:
            print("Dirección {direccion} no reconocida. Intente L, R o S")
        
              
        # Mover los motores trasero y delantero con la velocidad especificada
        self.front_motor.run_time(1000, duration, wait= False)
        self.rear_motor.run_time(1000, duration, wait=False)

        # Esperar la duración del movimiento
        while not self.front_motor.done() and not self.rear_motor.done():
            wait(10)

        # Detener todos los motores después de la duración
        self.rear_motor.stop()
        self.front_motor.stop()
        self.steering_motor.stop()

        print(f"Movimiento completado: Dirección: {direction} durante {duration} mseg")

    
    

            

