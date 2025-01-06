function drawInitandEnd(s, g, type)
    %% Función para dibujar el punto de inicio y fin
    % Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
    figure; hold on; grid on;
    xlabel('x [cm]'); ylabel('y [cm]');
    title(['Trayectorias de Dubins: ', type]);
   
    % Dibujar los puntos inicial y final
    plot(s(1), s(2), 'ro', 'LineWidth', 2);
    plot(g(1), g(2), 'bo', 'LineWidth', 2);

    %% Dibujo de los vectores de salida y llegada
    % Vector de salida
    quiver(s(1), s(2), cos(s(3)), sin(s(3)), 20, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    % Vector de llegada
    quiver(g(1), g(2), cos(g(3)), sin(g(3)), 20, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);

end

