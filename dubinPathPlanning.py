#dubinPathPlanning.py
#import umath
import math as umath

def check_angles(theta_arc, turn_type):
    """
    Ajusta el ángulo para que sea válido según el tipo de giro ('L' o 'R').

    Args:
        theta_arc: float - Ángulo inicial.
        turn_type: str - Tipo de giro ('L' para izquierda, 'R' para derecha).

    Returns:
        float: Ángulo ajustado.
    """
    if theta_arc < 0 and turn_type == 'L':
        theta_arc += 2 * umath.pi
    elif theta_arc > 0 and turn_type == 'R':
        theta_arc -= 2 * umath.pi
    return theta_arc

def get_tangents(x1, y1, r1, x2, y2, r2, path_type):
    """
    Calcula las tangentes entre dos círculos dados y selecciona una según el tipo.

    Args:
        x1, y1, r1: Coordenadas y radio del primer círculo.
        x2, y2, r2: Coordenadas y radio del segundo círculo.
        path_type: str - Tipo de trayectoria ('RSR', 'LSL', 'RSL', 'LSR').

    Returns:
        list: Tangente seleccionada según el tipo [x1, y1, x2, y2]. Vacío si no hay solución.
    """
    # Distancia cuadrada entre los centros
    d_sq = (x1 - x2) ** 2 + (y1 - y2) ** 2

    # Comprobación de existencia de tangentes válidas
    if d_sq <= (r1 - r2) ** 2:
        print("No existen tangentes válidas entre los círculos.")
        return []

    # Distancia entre los centros
    d = umath.sqrt(d_sq)

    # Vector unitario entre los centros
    vx = (x2 - x1) / d
    vy = (y2 - y1) / d

    # Inicialización de resultados
    tangents = []
    
    # Cálculo de las tangentes
    for sign1 in [+1, -1]:
        c = (r1 - sign1 * r2) / d

        # Verificar si es válido
        if c ** 2 > 1:
            continue

        # Altura del triángulo
        h = umath.sqrt(max(0, 1 - c ** 2))

        for sign2 in [+1, -1]:
            # Coordenadas del vector normal
            nx = vx * c - sign2 * h * vy
            ny = vy * c + sign2 * h * vx

            # Puntos de tangencia
            tangents.append([
                x1 + r1 * nx,  # x1 tangente
                y1 + r1 * ny,  # y1 tangente
                x2 + sign1 * r2 * nx,  # x2 tangente
                y2 + sign1 * r2 * ny   # y2 tangente
            ])

    # Verificar que el tipo sea válido
    valid_types = {'RSR': 0, 'LSL': 1, 'RSL': 2, 'LSR': 3}
    if path_type not in valid_types:
        raise ValueError("Tipo de trayectoria no válido. Tipos válidos: RSR, LSL, RSL, LSR.")

    # Seleccionar la tangente según el tipo
    index = valid_types[path_type]
    if index >= len(tangents):
        print("No se encontraron suficientes tangentes para el tipo solicitado.")
        return []

    return tangents[index]

def dubin_LRL(s, g, r_turn_min):
    """
    Calcula las longitudes de una trayectoria Dubins del tipo LRL.

    Args:
        s: List[float] - Punto inicial [x, y, theta]
        g: List[float] - Punto final [x, y, theta]
        r_turn_min: float - Radio mínimo de giro

    Returns:
        Tuple[List[float], str]: Longitudes de los segmentos y tipo de trayectoria
    """
    path_type = 'LRL'
    L = [-1, -1, -1]

    # Centros de los círculos C1 y C2
    c1 = [
        s[0] + r_turn_min * umath.cos(s[2] + umath.pi / 2),
        s[1] + r_turn_min * umath.sin(s[2] + umath.pi / 2)
    ]
    c2 = [
        g[0] + r_turn_min * umath.cos(g[2] + umath.pi / 2),
        g[1] + r_turn_min * umath.sin(g[2] + umath.pi / 2)
    ]

    # Distancia entre los centros
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    D = umath.sqrt(dx**2 + dy**2)

    # Verificar si es posible el camino LRL
    if D > 4 * r_turn_min * 0.95:
        print("Advertencia: La trayectoria LRL no es válida para los puntos dados.")
        return L, path_type

    # Cálculo del ángulo entre los círculos
    alpha = umath.atan2(dy, dx)
    beta = umath.acos(D / (4 * r_turn_min))
    theta = alpha + beta

    # Centro del círculo intermedio C3
    c3 = [
        c1[0] + 2 * r_turn_min * umath.cos(theta),
        c1[1] + 2 * r_turn_min * umath.sin(theta)
    ]

    # Puntos tangentes
    pt1 = [
        c3[0] + r_turn_min * (c1[0] - c3[0]) / umath.sqrt((c1[0] - c3[0])**2 + (c1[1] - c3[1])**2),
        c3[1] + r_turn_min * (c1[1] - c3[1]) / umath.sqrt((c1[0] - c3[0])**2 + (c1[1] - c3[1])**2)
    ]
    pt2 = [
        c3[0] + r_turn_min * (c2[0] - c3[0]) / umath.sqrt((c2[0] - c3[0])**2 + (c2[1] - c3[1])**2),
        c3[1] + r_turn_min * (c2[1] - c3[1]) / umath.sqrt((c2[0] - c3[0])**2 + (c2[1] - c3[1])**2)
    ]

    # Tramo 1: Arco inicial (izquierda)
    theta1_start = umath.atan2(s[1] - c1[1], s[0] - c1[0])
    theta1_end = umath.atan2(pt1[1] - c1[1], pt1[0] - c1[0])
    theta1_arc = check_angles(theta1_end - theta1_start, 'L')
    L[0] = r_turn_min * abs(theta1_arc)

    # Tramo 2: Segundo arco (derecho)
    theta2_start = umath.atan2(pt1[1] - c3[1], pt1[0] - c3[0])
    theta2_end = umath.atan2(pt2[1] - c3[1], pt2[0] - c3[0])
    theta2_arc = check_angles(theta2_end - theta2_start, 'R')
    L[1] = r_turn_min * abs(theta2_arc)

    # Tramo 3: Arco final (izquierda)
    theta3_start = umath.atan2(pt2[1] - c2[1], pt2[0] - c2[0])
    theta3_end = umath.atan2(g[1] - c2[1], g[0] - c2[0])
    theta3_arc = check_angles(theta3_end - theta3_start, 'L')
    L[2] = r_turn_min * abs(theta3_arc)

    return L, path_type

def dubin_LSL(s, g, r_turn_min):
    """
    Calcula las longitudes de una trayectoria Dubins del tipo LSL.

    Args:
        s: List[float] - Punto inicial [x, y, theta]
        g: List[float] - Punto final [x, y, theta]
        r_turn_min: float - Radio mínimo de giro

    Returns:
        Tuple[List[float], str]: Longitudes de los segmentos y tipo de trayectoria
    """
    path_type = 'LSL'
    L = [-1, -1, -1]

    # Centros de los círculos C1 y C2
    c1 = [
        s[0] + r_turn_min * umath.cos(s[2] + umath.pi / 2),
        s[1] + r_turn_min * umath.sin(s[2] + umath.pi / 2)
    ]
    c2 = [
        g[0] + r_turn_min * umath.cos(g[2] + umath.pi / 2),
        g[1] + r_turn_min * umath.sin(g[2] + umath.pi / 2)
    ]

    # Distancia entre los centros
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    D = umath.sqrt(dx**2 + dy**2)

    # Verificar si es posible el camino LSL
    if D < 2 * r_turn_min:
        print("Advertencia: La trayectoria LSL no es válida para los puntos dados.")
        return L, path_type

    # Obtener las tangentes correspondientes
    tangents = get_tangents(c1[0], c1[1], r_turn_min, c2[0], c2[1], r_turn_min, path_type)
    if tangents is None:
        print("Advertencia: No se encontraron tangentes válidas para el tipo LSL.")
        return L, path_type

    # Tramo 1: Arco inicial (izquierda)
    theta1_start = umath.atan2(s[1] - c1[1], s[0] - c1[0])
    theta1_end = umath.atan2(tangents[1] - c1[1], tangents[0] - c1[0])
    theta1_arc = check_angles(theta1_end - theta1_start, 'L')
    L[0] = r_turn_min * abs(theta1_arc)

    # Tramo 2: Segmento recto
    L[1] = umath.sqrt((tangents[2] - tangents[0])**2 + (tangents[3] - tangents[1])**2)

    # Tramo 3: Arco final (izquierda)
    theta2_start = umath.atan2(tangents[3] - c2[1], tangents[2] - c2[0])
    theta2_end = umath.atan2(g[1] - c2[1], g[0] - c2[0])
    theta2_arc = check_angles(theta2_end - theta2_start, 'L')
    L[2] = r_turn_min * abs(theta2_arc)

    return L, path_type

def dubin_LSR(s, g, r_turn_min):
    """
    Calcula las longitudes de una trayectoria Dubins del tipo LSR.

    Args:
        s (tuple): Estado inicial (x, y, theta).
        g (tuple): Estado final (x, y, theta).
        r_turn_min (float): Radio mínimo de giro.

    Returns:
        tuple: (L, type), donde:
            L (list): Longitudes de los tres segmentos (arco-recta-arco). Si no es válida, [-1, -1, -1].
            type (str): Tipo de trayectoria ('LSR').
    """
    path_type = "LSR"
    L = [0.0, 0.0, 0.0]

    # Centros de los círculos C1 y C2
    c1 = (
        s[0] + r_turn_min * umath.cos(s[2] + umath.pi / 2),
        s[1] + r_turn_min * umath.sin(s[2] + umath.pi / 2)
    )
    c2 = (
        g[0] + r_turn_min * umath.cos(g[2] - umath.pi / 2),
        g[1] + r_turn_min * umath.sin(g[2] - umath.pi / 2)
    )

    # Distancia entre los centros
    D = umath.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

    # Verificar si es posible el camino LSR
    if D < 2 * r_turn_min:
        print("La trayectoria LSR no es válida para los puntos dados.")
        return ([-1, -1, -1], path_type)

    # Obtener las tangentes correspondientes
    tangents = get_tangents(c1[0], c1[1], r_turn_min, c2[0], c2[1], r_turn_min, path_type)
    if not tangents:
        print("No se encontraron tangentes válidas para el tipo LSR.")
        return ([-1, -1, -1], path_type)

    # Tramo 1: Arco inicial (izquierda)
    theta1_start = umath.atan2(s[1] - c1[1], s[0] - c1[0])
    theta1_end = umath.atan2(tangents[1] - c1[1], tangents[0] - c1[0])
    theta1_arc = theta1_end - theta1_start
    theta1_arc = check_angles(theta1_arc, 'L')  # Corregir el ángulo para arco izquierdo
    L[0] = r_turn_min * abs(theta1_arc)

    # Tramo 2: Segmento recto
    L[1] = umath.sqrt((tangents[2] - tangents[0])**2 + (tangents[3] - tangents[1])**2)

    # Tramo 3: Arco final (derecha)
    theta2_start = umath.atan2(tangents[3] - c2[1], tangents[2] - c2[0])
    theta2_end = umath.atan2(g[1] - c2[1], g[0] - c2[0])
    theta2_arc = theta2_end - theta2_start
    theta2_arc = check_angles(theta2_arc, 'R')  # Corregir el ángulo para arco derecho
    L[2] = r_turn_min * abs(theta2_arc)

    return (L, path_type)

def dubin_RLR(s, g, r_turn_min):
    """
    Calcula las longitudes de una trayectoria Dubins del tipo RLR.

    Args:
        s (tuple): Estado inicial (x, y, theta).
        g (tuple): Estado final (x, y, theta).
        r_turn_min (float): Radio mínimo de giro.

    Returns:
        tuple: (L, type), donde:
            L (list): Longitudes de los tres segmentos (arco-arco-arco). Si no es válida, [-1, -1, -1].
            type (str): Tipo de trayectoria ('RLR').
    """
    path_type = "RLR"
    L = [0.0, 0.0, 0.0]

    # Centros de los círculos C1 y C2
    c1 = (
        s[0] + r_turn_min * umath.cos(s[2] - umath.pi / 2),
        s[1] + r_turn_min * umath.sin(s[2] - umath.pi / 2)
    )
    c2 = (
        g[0] + r_turn_min * umath.cos(g[2] - umath.pi / 2),
        g[1] + r_turn_min * umath.sin(g[2] - umath.pi / 2)
    )

    # Distancia entre los centros
    D = umath.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

    # Verificar si es posible el camino RLR
    if D > 4 * r_turn_min * 0.95:
        print("La trayectoria RLR no es válida para los puntos dados.")
        return ([-1, -1, -1], path_type)

    # Cálculo del ángulo entre los círculos
    alpha = umath.atan2(c2[1] - c1[1], c2[0] - c1[0])
    beta = umath.acos(D / (4 * r_turn_min))
    theta = alpha - beta

    # Centro del círculo intermedio C3
    c3 = (
        c1[0] + 2 * r_turn_min * umath.cos(theta),
        c1[1] + 2 * r_turn_min * umath.sin(theta)
    )

    # Puntos tangentes
    pt1 = (
        c3[0] + r_turn_min * (c1[0] - c3[0]) / umath.sqrt((c1[0] - c3[0])**2 + (c1[1] - c3[1])**2),
        c3[1] + r_turn_min * (c1[1] - c3[1]) / umath.sqrt((c1[0] - c3[0])**2 + (c1[1] - c3[1])**2)
    )
    pt2 = (
        c3[0] + r_turn_min * (c2[0] - c3[0]) / umath.sqrt((c2[0] - c3[0])**2 + (c2[1] - c3[1])**2),
        c3[1] + r_turn_min * (c2[1] - c3[1]) / umath.sqrt((c2[0] - c3[0])**2 + (c2[1] - c3[1])**2)
    )

    # Tramo 1: Arco inicial (derecho)
    theta1_start = umath.atan2(s[1] - c1[1], s[0] - c1[0])
    theta1_end = umath.atan2(pt1[1] - c1[1], pt1[0] - c1[0])
    theta1_arc = check_angles(theta1_end - theta1_start, 'R')
    L[0] = r_turn_min * abs(theta1_arc)

    # Tramo 2: Segundo arco (izquierdo)
    theta2_start = umath.atan2(pt1[1] - c3[1], pt1[0] - c3[0])
    theta2_end = umath.atan2(pt2[1] - c3[1], pt2[0] - c3[0])
    theta2_arc = check_angles(theta2_end - theta2_start, 'L')
    L[1] = r_turn_min * abs(theta2_arc)

    # Tramo 3: Arco final (derecho)
    theta3_start = umath.atan2(pt2[1] - c2[1], pt2[0] - c2[0])
    theta3_end = umath.atan2(g[1] - c2[1], g[0] - c2[0])
    theta3_arc = check_angles(theta3_end - theta3_start, 'R')
    L[2] = r_turn_min * abs(theta3_arc)

    return (L, path_type)

def dubin_RSL(s, g, r_turn_min):
    """
    Calcula las longitudes de una trayectoria Dubins del tipo RSL.

    Args:
        s (tuple): Estado inicial (x, y, theta).
        g (tuple): Estado final (x, y, theta).
        r_turn_min (float): Radio mínimo de giro.

    Returns:
        tuple: (L, type), donde:
            L (list): Longitudes de los tres segmentos (arco-recta-arco). Si no es válida, [-1, -1, -1].
            type (str): Tipo de trayectoria ('RSL').
    """
    path_type = "RSL"
    L = [0.0, 0.0, 0.0]

    # Centros de los círculos C1 y C2
    c1 = (
        s[0] + r_turn_min * umath.cos(s[2] - umath.pi / 2),
        s[1] + r_turn_min * umath.sin(s[2] - umath.pi / 2)
    )
    c2 = (
        g[0] + r_turn_min * umath.cos(g[2] + umath.pi / 2),
        g[1] + r_turn_min * umath.sin(g[2] + umath.pi / 2)
    )

    # Distancia entre los centros
    D = umath.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

    # Verificar si es posible el camino RSL
    if D < 2 * r_turn_min:
        print("La trayectoria RSL no es válida para los puntos dados.")
        return ([-1, -1, -1], path_type)

    # Obtener las tangentes correspondientes
    tangents = get_tangents(c1[0], c1[1], r_turn_min, c2[0], c2[1], r_turn_min, path_type)
    if not tangents:
        print("No se encontraron tangentes válidas para el tipo RSL.")
        return ([-1, -1, -1], path_type)

    # Tramo 1: Arco inicial (derecho)
    theta1_start = umath.atan2(s[1] - c1[1], s[0] - c1[0])
    theta1_end = umath.atan2(tangents[1] - c1[1], tangents[0] - c1[0])
    theta1_arc = check_angles(theta1_end - theta1_start, 'R')
    L[0] = r_turn_min * abs(theta1_arc)

    # Tramo 2: Segmento recto
    L[1] = umath.sqrt((tangents[2] - tangents[0])**2 + (tangents[3] - tangents[1])**2)

    # Tramo 3: Arco final (izquierdo)
    theta2_start = umath.atan2(tangents[3] - c2[1], tangents[2] - c2[0])
    theta2_end = umath.atan2(g[1] - c2[1], g[0] - c2[0])
    theta2_arc = check_angles(theta2_end - theta2_start, 'L')
    L[2] = r_turn_min * abs(theta2_arc)

    return (L, path_type)

def dubin_RSR(s, g, r_turn_min):
    """
    Calcula las longitudes de una trayectoria Dubins del tipo RSR.

    Args:
        s (tuple): Estado inicial (x, y, theta).
        g (tuple): Estado final (x, y, theta).
        r_turn_min (float): Radio mínimo de giro.

    Returns:
        tuple: (L, type), donde:
            L (list): Longitudes de los tres segmentos (arco-recta-arco). Si no es válida, [-1, -1, -1].
            type (str): Tipo de trayectoria ('RSR').
    """
    path_type = "RSR"
    L = [0.0, 0.0, 0.0]

    # Centros de los círculos C1 y C2
    c1 = (
        s[0] + r_turn_min * umath.cos(s[2] - umath.pi / 2),
        s[1] + r_turn_min * umath.sin(s[2] - umath.pi / 2)
    )
    c2 = (
        g[0] + r_turn_min * umath.cos(g[2] - umath.pi / 2),
        g[1] + r_turn_min * umath.sin(g[2] - umath.pi / 2)
    )

    # Distancia entre los centros
    D = umath.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

    # Verificar si es posible el camino RSR
    if D < 2 * r_turn_min:
        print("La trayectoria RSR no es válida para los puntos dados.")
        return ([-1, -1, -1], path_type)

    # Obtener las tangentes correspondientes
    tangents = get_tangents(c1[0], c1[1], r_turn_min, c2[0], c2[1], r_turn_min, path_type)
    if not tangents:
        print("No se encontraron tangentes válidas para el tipo RSR.")
        return ([-1, -1, -1], path_type)

    # Tramo 1: Arco inicial (derecho)
    theta1_start = umath.atan2(s[1] - c1[1], s[0] - c1[0])
    theta1_end = umath.atan2(tangents[1] - c1[1], tangents[0] - c1[0])
    theta1_arc = check_angles(theta1_end - theta1_start, 'R')
    L[0] = r_turn_min * abs(theta1_arc)

    # Tramo 2: Segmento recto
    L[1] = umath.sqrt((tangents[2] - tangents[0])**2 + (tangents[3] - tangents[1])**2)

    # Tramo 3: Arco final (derecho)
    theta2_start = umath.atan2(tangents[3] - c2[1], tangents[2] - c2[0])
    theta2_end = umath.atan2(g[1] - c2[1], g[0] - c2[0])
    theta2_arc = check_angles(theta2_end - theta2_start, 'R')
    L[2] = r_turn_min * abs(theta2_arc)

    return (L, path_type)

def dubin_calculation(s, g, r_turn_min):
    # Parámetros iniciales
    #s  Postura inicial
    #g  Postura final
    #r_turn_min  Radio mínimo de giro en cm
    

    # Inicialización de variables
    L = [[-1, -1, -1] for _ in range(6)]  # Longitudes iniciales
    types = [''] * 6  # Tipos de trayectoria

    # Cálculo de trayectorias
    L[0], types[0] = dubin_RSR(s, g, r_turn_min)
    L[1], types[1] = dubin_LSL(s, g, r_turn_min)
    L[2], types[2] = dubin_LSR(s, g, r_turn_min)
    L[3], types[3] = dubin_RSL(s, g, r_turn_min)
    L[4], types[4] = dubin_RLR(s, g, r_turn_min)
    L[5], types[5] = dubin_LRL(s, g, r_turn_min)

    # Selección de la mejor trayectoria
    best_cost = float('inf')
    L_selected = [float('inf'), float('inf'), float('inf')]
    type_selected = ''

    for i in range(6):
        if all(val != -1 for val in L[i]):  # Verifica si la trayectoria es válida
            cost = sum(L[i])  # Calcula el costo total
            if cost < best_cost:
                best_cost = cost
                L_selected = L[i]
                type_selected = types[i]

    return L_selected, type_selected