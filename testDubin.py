from dubinPathPlanning import dubin_calculation
import math as umath
from audiConstants import L_car, psi_max, v

def dubin_test():
    # Parámetros iniciales
    s = [0, 0, umath.radians(0)]  # Postura inicial
    g = [24, 120, umath.radians(45)]  # Postura final
    
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
    dubin_test()
    print("Terminado")