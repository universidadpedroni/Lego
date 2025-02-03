function drawDubinTrajectory2(s, g, r_turn_min, type, L, h, v, L_car, psi_max)
%% Función generalizada para todas las trayectorias CSC y CCC
% Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
decimation_constant = 40;
vector_size = 15;

RIGHT = -1;
LEFT = 1;
STRAIGTH = 0;

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

time_pause = 0.001;
delta_s = h * v; % Distancia recorrida por paso
hold on; % Mantén el gráfico para dibujar iterativamente
for i = 1:3 % Hay tres tramos
    if(type(i) == 'R')
        steering = RIGHT;
    elseif (type(i) == 'L')
        steering = LEFT;
    else
        steering = STRAIGTH;
    end
    
    decimation_index = 0;

    encoder = 0;
    while(encoder <= L(i))
        x_new = x + delta_s * cos(theta);
        y_new = y + delta_s * sin(theta);
        theta = theta + steering * delta_s / r_turn_min; % Cambio angular basado en distancia
        plot([x, x_new], [y, y_new], 'g', 'LineWidth', 1);
        % Simulación del avance del encoder del vehículo.
        encoder = encoder + delta_s;
        % Dibujar el vehículo y mostrar información        
        if mod(decimation_index, decimation_constant) == 0
            quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
            drawCar(x, y, theta, L_car, -psi_max);
            fprintf("Encoder: %.2f \t L(%d): %.2f\n", encoder, i, L(i));
            
        end
            
        x = x_new; 
        y = y_new;
        decimation_index = decimation_index + 1;
        pause(time_pause);


    end
    
end
% Dibujar el último auto
quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
drawCar(x, y, theta, L_car, -psi_max);
fprintf("Encoder: %.2f \t L(%d): %.2f\n", encoder, i, L(i));

disp(['Animación completada. Trayectoria tipo ', type]);

end
