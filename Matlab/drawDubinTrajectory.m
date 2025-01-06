function drawDubinTrajectory(s, g, r_turn_min, type, L, h, v)
%% Función generalizada para todas las trayectorias CSC y CCC
% Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.

% Centros de los círculos
if type(1) == 'R'
    c1 = [s(1) + r_turn_min * cos(s(3) - pi/2), ...
        s(2) + r_turn_min * sin(s(3) - pi/2)];
else % 'L'
    c1 = [s(1) + r_turn_min * cos(s(3) + pi/2), ...
        s(2) + r_turn_min * sin(s(3) + pi/2)];
end

if type(3) == 'R'
    c2 = [g(1) + r_turn_min * cos(g(3) - pi/2), ...
        g(2) + r_turn_min * sin(g(3) - pi/2)];
else % 'L'
    c2 = [g(1) + r_turn_min * cos(g(3) + pi/2), ...
        g(2) + r_turn_min * sin(g(3) + pi/2)];
end

% Dibujar los círculos de giro mínimo
viscircles(c1, r_turn_min, 'Color', 'b', 'LineStyle', '--');
viscircles(c2, r_turn_min, 'Color', 'r', 'LineStyle', '--');




%% Dibujo de la trayectoria
% Inicialización de las variables
x = s(1); y = s(2); theta = s(3);

% Aquí se va a graficar la trayectoria
trajectory_x = x;  % Inicialización para la animación
trajectory_y = y;  % Inicialización para la animación
time_pause = 0.05;
delta_s = h * v; % Distancia recorrida por paso
for i = 1:3 % Hay tres tramos
    if type(i) == 'R'   % Tramo a la derecha
        for j = 1:floor(L(i) / delta_s)
            x = x + delta_s * cos(theta);
            y = y + delta_s * sin(theta);
            theta = theta - delta_s / r_turn_min; % Cambio angular basado en distancia
            trajectory_x = [trajectory_x, x];
            trajectory_y = [trajectory_y, y];
            plot(trajectory_x, trajectory_y, 'g', 'LineWidth', 2);
            pause(time_pause); % Mantén el intervalo de tiempo para la animación
        end
    elseif type(i) == 'L' % Tramo a la izquierda
        for j = 1:floor(L(i) / delta_s)
            x = x + delta_s * cos(theta);
            y = y + delta_s * sin(theta);
            theta = theta + delta_s / r_turn_min;
            trajectory_x = [trajectory_x, x];
            trajectory_y = [trajectory_y, y];
            plot(trajectory_x, trajectory_y, 'g', 'LineWidth', 2);
            pause(time_pause);
        end
    else    % Tramo recto
        for j = 1:floor(L(i) / delta_s)
            x = x + delta_s * cos(theta);
            y = y + delta_s * sin(theta);
            trajectory_x = [trajectory_x, x];
            trajectory_y = [trajectory_y, y];
            plot(trajectory_x, trajectory_y, 'g', 'LineWidth', 2);
            pause(time_pause);
        end
    end
end

disp(['Animación completada. Trayectoria tipo ', type]);
end