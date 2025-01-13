function drawInitandEnd(s, g, type)
    %% Función para dibujar el punto de inicio y fin
    % Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
    figure('units','normalized','outerposition',[0 0 1 1]); 
    hold on; grid on;
    xlabel('x [cm]'); ylabel('y [cm]');
    title(['Trayectoria de Dubins: ', type]);
   
    % Dibujar los puntos inicial y final
    plot(s(1), s(2), 'ro', 'LineWidth', 2);
    plot(g(1), g(2), 'ro', 'LineWidth', 2);

    %% Dibujo de los vectores de salida y llegada
    % Vector de salida
    quiver(s(1), s(2), cos(s(3)), sin(s(3)), 20, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    text(s(1) + 1, s(2) + 1, "Postura inicial");
    % Vector de llegada
    quiver(g(1), g(2), cos(g(3)), sin(g(3)), 20, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    text(g(1) + 1, g(2) + 1, "Postura final");
end

