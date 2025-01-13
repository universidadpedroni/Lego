function drawDubinTrajectory(s, g, r_turn_min, type, L, h, v, L_car, psi_max)
%% Función generalizada para todas las trayectorias CSC y CCC
% Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
vector_decimation = 40;
vector_size = 15;

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
viscircles(c2, r_turn_min, 'Color', 'b', 'LineStyle', '--');

%% Dibujo de la trayectoria
x = s(1); 
y = s(2); 
theta = s(3);

time_pause = 0.05;
delta_s = h * v; % Distancia recorrida por paso
hold on; % Mantén el gráfico para dibujar iterativamente
for i = 1:3 % Hay tres tramos
    if type(i) == 'R'   % Tramo a la derecha
        for j = 1:floor(L(i) / delta_s)
            x_new = x + delta_s * cos(theta);
            y_new = y + delta_s * sin(theta);
            theta = theta - delta_s / r_turn_min; % Cambio angular basado en distancia
            plot([x, x_new], [y, y_new], 'g', 'LineWidth', 1);
            
            if mod(j, vector_decimation) == 0
                quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
                drawCar(x, y, theta, L_car, -psi_max);
            end
            
            x = x_new; 
            y = y_new;
            pause(time_pause);
        end
    elseif type(i) == 'L' % Tramo a la izquierda
        for j = 1:floor(L(i) / delta_s)
            x_new = x + delta_s * cos(theta);
            y_new = y + delta_s * sin(theta);
            theta = theta + delta_s / r_turn_min;
            plot([x, x_new], [y, y_new], 'g', 'LineWidth', 1);
            
            if mod(j, vector_decimation) == 0
                quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
                drawCar(x, y, theta, L_car, 0)
            end
            
            x = x_new; 
            y = y_new;
            pause(time_pause);
        end
    else    % Tramo recto
        for j = 1:floor(L(i) / delta_s)
            x_new = x + delta_s * cos(theta);
            y_new = y + delta_s * sin(theta);
            plot([x, x_new], [y, y_new], 'g', 'LineWidth', 1);
            
            if mod(j, vector_decimation) == 0
                quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
                drawCar(x, y, theta, L_car, psi_max);
            end
            
            x = x_new; 
            y = y_new;
            pause(time_pause);
        end
    end
end

disp(['Animación completada. Trayectoria tipo ', type]);
end
