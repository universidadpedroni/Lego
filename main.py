#main.py
#  
# Para cargar el script en el hub presionar F5. Se descargará automáticamente.
#Puertos: 
# A: Eje trasero
# B: Eje delantero
# D: Dirección
# Hub address: 02:C2:BD:6A:3D:3D
#REVISAR:
#https://github.com/orgs/pybricks/discussions/162
#https://pybricks.com/project/technic-42160-xbox/
#https://github.com/orgs/pybricks/discussions/1843
# Para usar el teclado como entrada: https://pybricks.com/projects/sets/technic/42099-off-roader/keyboard-remote/

#from umath import sin, pi
#from pybricks.tools import wait, StopWatch
from vehicle import Vehicle
from dubinPathPlanning import dubin_calculation
from audiConstants import L_car, psi_max, v

from pybricks.hubs import TechnicHub
from pybricks.parameters import Color
from pybricks import version
import usys
from pybricks.tools import wait
import umath

hub = TechnicHub()
audi = Vehicle()

def get_path():
     # Parámetros iniciales
    s = [0, 0, umath.radians(0)]  # Postura inicial
    g = [24, 0, umath.radians(90)]  # Postura final
    
    r_turn_min = L_car / umath.sin(umath.radians(psi_max))  # Radio mínimo de giro en cm


    L_selected, type_selected = dubin_calculation(s, g, r_turn_min)
    
    # Cálculo de tiempos por tramo
    times = [l / v for l in L_selected]  # Tiempo en segundos por tramo

    # Mostrar resultados
    
    print(f"Trayectoria seleccionada: {type_selected}")
    print(f"Longitudes: {', '.join([f'{l:.2f}' for l in L_selected])}")
    print(f"Tiempos por tramo (ms): {', '.join([f'{t * 1000:.2f}' for t in times])}")
    print(f"Tiempo total (s): {sum(times):.2f}")

if __name__ == "__main__":
   
    try:
        hub.light.on(Color.RED)
        print(hub.system.name())
        print(version)
        print(usys.implementation)
        print("Si la batería está baja no sube el programa")
        audi.setup()
        audi.calibrate_steering_motor()
        print(hub.battery.voltage() / 1000, hub.battery.current(),  hub.imu.settings(), hub.imu.ready(), hub.imu.stationary())
        get_path()
        
       
    except Exception as e:
        print(f"Error durante la ejecución: {e}")
    finally:
        # Asegúrate de limpiar recursos aquí si es necesario
        print("Finalizando el programa correctamente.")
        hub.light.on(Color.GREEN)
        wait(3000)
   
  


