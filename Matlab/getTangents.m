function selected_tangent = getTangents(x1, y1, r1, x2, y2, r2, type)
    % Calcula las tangentes entre dos círculos dados y selecciona una según el tipo.
    %
    % Entradas:
    % x1, y1, r1 - Coordenadas y radio del primer círculo.
    % x2, y2, r2 - Coordenadas y radio del segundo círculo.
    % type       - Tipo de trayectoria ('RSR', 'LSL', 'RSL', 'LSR', 'RLR', 'LRL').
    %
    % Salidas:
    % selected_tangent - Tangente seleccionada según el tipo [x1, y1, x2, y2].
    %                    Si no hay solución, retorna un vector vacío [].

    % Distancia cuadrada entre los centros de los círculos
    d_sq = (x1 - x2)^2 + (y1 - y2)^2;

    % Comprobación de que existe al menos una tangente
    if d_sq <= (r1 - r2)^2
        warning('No existen tangentes válidas entre los círculos.');
        selected_tangent = [];
        return;
    end

    % Distancia entre los centros
    d = sqrt(d_sq);

    % Vector unitario entre los centros
    vx = (x2 - x1) / d;
    vy = (y2 - y1) / d;

    % Inicialización de resultados
    tangents = zeros(4, 4); % Máximo 4 tangentes
    i = 1;

    % Cálculo de las tangentes
    for sign1 = [+1, -1]
        c = (r1 - sign1 * r2) / d;

        % Comprobar si es válido (no puede ser mayor que 1)
        if c^2 > 1
            continue;
        end

        % Altura del triángulo
        h = sqrt(max(0, 1 - c^2));

        for sign2 = [+1, -1]
            % Coordenadas del vector normal
            nx = vx * c - sign2 * h * vy;
            ny = vy * c + sign2 * h * vx;

            % Puntos de tangencia
            tangents(i, 1) = x1 + r1 * nx; % x1
            tangents(i, 2) = y1 + r1 * ny; % y1
            tangents(i, 3) = x2 + sign1 * r2 * nx; % x2
            tangents(i, 4) = y2 + sign1 * r2 * ny; % y2
            i = i + 1;
        end
    end

    % Eliminar filas vacías
    tangents = tangents(1:i-1, :);

    % Verificar que el tipo sea válido
    valid_types = {'RSR', 'LSL', 'RSL', 'LSR', 'RLR', 'LRL'};
    if ~ismember(type, valid_types)
        error('Tipo de trayectoria no válido. Tipos válidos: %s', strjoin(valid_types, ', '));
    end

    % Seleccionar la tangente según el tipo
    try
        switch type
            case 'RSR'
                selected_tangent = tangents(1, :);
            case 'LSL'
                selected_tangent = tangents(2, :);
            case 'RSL'
                selected_tangent = tangents(3, :);
            case 'LSR'
                selected_tangent = tangents(4, :);
            % case 'RLR'
            %     selected_tangent = tangents(3, :);
            % case 'LRL'
            %     selected_tangent = tangents(4, :);
        end
    catch
        warning('No se encontraron suficientes tangentes para el tipo solicitado.');
        selected_tangent = [];
    end
end
