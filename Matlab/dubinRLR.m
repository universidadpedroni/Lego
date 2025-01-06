function [L, type] = dubinRLR(s, g, r_turn_min)
    % Calcula las longitudes de una trayectoria Dubins del tipo RLR.
    type = 'RLR';
    L = zeros(1, 3);

    % Centros de los círculos C1 y C2
    c1 = [s(1) + r_turn_min * cos(s(3) - pi/2), ...
          s(2) + r_turn_min * sin(s(3) - pi/2)];
    c2 = [g(1) + r_turn_min * cos(g(3) - pi/2), ...
          g(2) + r_turn_min * sin(g(3) - pi/2)];

    % Distancia entre los centros
    D = sqrt((c2(1) - c1(1))^2 + (c2(2) - c1(2))^2);

    % Verificar si es posible el camino RLR. En rigor se debe verificar que
    % D > 4 * r_turn_min, pero en el límite puede fallar
    if  D > 4 * r_turn_min * 0.95
        warning('La trayectoria RLR no es válida para los puntos dados.');
        L = [-1 -1 -1];
        return
    end

    % Cálculo del ángulo entre los círculos
    alpha = atan2(c2(2) - c1(2), c2(1) - c1(1));
    beta = acos(D / (4 * r_turn_min));
    theta = alpha - beta;

    % Centro del círculo intermedio C3
    c3 = [c1(1) + 2 * r_turn_min * cos(theta), ...
          c1(2) + 2 * r_turn_min * sin(theta)];

    % Puntos tangentes
    pt1 = c3 + r_turn_min * (c1 - c3) / norm(c1 - c3);
    pt2 = c3 + r_turn_min * (c2 - c3) / norm(c2 - c3);

    % Tramo 1: Arco inicial (derecho)
    theta1_start = atan2(s(2) - c1(2), s(1) - c1(1));
    theta1_end = atan2(pt1(2) - c1(2), pt1(1) - c1(1));
    theta1_arc = checkAngles(theta1_end - theta1_start, 'R');
    L(1) = r_turn_min * abs(theta1_arc);

    % Tramo 2: Segundo arco (izquierdo)
    theta2_start = atan2(pt1(2) - c3(2), pt1(1) - c3(1));
    theta2_end = atan2(pt2(2) - c3(2), pt2(1) - c3(1));
    theta2_arc = checkAngles(theta2_end - theta2_start, 'L');
    L(2) = r_turn_min * abs(theta2_arc);

    % Tramo 3: Arco final (derecho)
    theta3_start = atan2(pt2(2) - c2(2), pt2(1) - c2(1));
    theta3_end = atan2(g(2) - c2(2), g(1) - c2(1));
    theta3_arc = checkAngles(theta3_end - theta3_start, 'R');
    L(3) = r_turn_min * abs(theta3_arc);

    % Mostrar resultados
    disp(['Longitud del arco 1: ', num2str(L(1)), ' cm']);
    disp(['Longitud del arco 2: ', num2str(L(2)), ' cm']);
    disp(['Longitud del arco 3: ', num2str(L(3)), ' cm']);
end
