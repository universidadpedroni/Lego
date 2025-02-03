import umath

def infoTrayectoria(s, g, L, path_type, estimated_driving_time):
    """
    Muestra información sobre una trayectoria calculada.

    Parámetros:
    - s: postura inicial [x, y, theta (en radianes)].
    - g: postura final [x, y, theta (en radianes)].
    - L: lista de longitudes de los segmentos (en cm).
    - path_type: tipo de trayectoria.
    - estimated_driving_time: tiempo estimado para recorrer la trayectoria (en segundos).
    """
    # Generar las longitudes formateadas manualmente
    longitudes_formateadas = ", ".join(["{:.2f}".format(length) for length in L])

    # Convertir ángulos de radianes a grados para s y g
    s_formateado = "({:.2f}, {:.2f}, {:.2f}°)".format(s[0], s[1], umath.degrees(s[2]))
    g_formateado = "({:.2f}, {:.2f}, {:.2f}°)".format(g[0], g[1], umath.degrees(g[2]))

    # Construir el mensaje
    mensaje = (
        "Postura inicial (s): {}\n"
        "Postura final (g): {}\n"
        "Longitudes de los segmentos: {} cm\n"
        "Tipo de trayectoria: {}\n"
        "Longitud total: {:.2f} cm\n"
        "Tiempo total estimado: {:.2f} s"
    ).format(s_formateado, g_formateado, longitudes_formateadas, path_type, sum(L), estimated_driving_time)

    # Imprimir el mensaje
    print(mensaje)


def infoControl(elapsed_time, position_reference, position_vehicle, position_error, position_command,
                              steering_reference, steering_vehicle, steering_error, steering_command):
    mensaje = (
        "E_Tim: {:.1f}s | P_Ref: {:.1f}, P_Veh: {:.1f}, P_Err: {:.1f}, P_Com: {:.0f}| "
        "St_Ref: {:.1f}, St_Veh: {:.1f}, St_Err: {:.1f}, St_Com: {:.0f} "
    ).format(elapsed_time, position_reference, position_vehicle, position_error, position_command,
                           umath.degrees(steering_reference), steering_vehicle, steering_error, steering_command)

    print (mensaje)
        