function [L, type] = dubinRSL(s, g, r_turn_min)
    % Calcula las longitudes de una trayectoria Dubins del tipo RSL.
    type = 'RSL';
    L = zeros(1, 3);

    % Centros de los círculos C1 y C2
    c1 = [s(1) + r_turn_min * cos(s(3) - pi/2), ...
          s(2) + r_turn_min * sin(s(3) - pi/2)];
    c2 = [g(1) + r_turn_min * cos(g(3) + pi/2), ...
          g(2) + r_turn_min * sin(g(3) + pi/2)];
    
    % Distancia entre los centros
    D = sqrt((c2(1) - c1(1))^2 + (c2(2) - c1(2))^2);

    % Verificar si es posible el camino LSL.En rigor se debe verificar que
    % D < 2 * r_turn_min, pero en el límite puede fallar
    if D < 2 * r_turn_min
        warning('La trayectoria RSL no es válida para los puntos dados.');
        L = [-1 -1 -1];
        return
    end

    % Obtener las tangentes correspondientes
    tangents = getTangents(c1(1), c1(2), r_turn_min, ...
                           c2(1), c2(2), r_turn_min, type);
    if isempty(tangents)
        warning('No se encontraron tangentes válidas para el tipo RSL.');
        L = [-1 -1 -1];
        return
    end

    % Tramo 1: Arco inicial (derecha)
    theta1_start = atan2(s(2) - c1(2), s(1) - c1(1));
    theta1_end = atan2(tangents(2) - c1(2), tangents(1) - c1(1));
    theta1_arc = theta1_end - theta1_start;
    theta1_arc = checkAngles(theta1_arc, 'R'); % Corregir el ángulo para arco derecho
    L(1) = r_turn_min * abs(theta1_arc);

    % Tramo 2: Segmento recto
    L(2) = sqrt((tangents(3) - tangents(1))^2 + ...
                (tangents(4) - tangents(2))^2);

    % Tramo 3: Arco final (izquierda)
    theta2_start = atan2(tangents(4) - c2(2), tangents(3) - c2(1));
    theta2_end = atan2(g(2) - c2(2), g(1) - c2(1));
    theta2_arc = theta2_end - theta2_start;
    theta2_arc = checkAngles(theta2_arc, 'L'); % Corregir el ángulo para arco izquierdo
    L(3) = r_turn_min * abs(theta2_arc);

    % Mostrar resultados
    % disp(['Longitud del arco 1: ', num2str(L(1)), ' cm']);
    % disp(['Longitud de la recta: ', num2str(L(2)), ' cm']);
    % disp(['Longitud del arco 2: ', num2str(L(3)), ' cm']);
end
