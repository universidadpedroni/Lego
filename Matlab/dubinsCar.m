%% DUBINS CAR
close all; clear all; clc
disp('https://gieseanw.wordpress.com/wp-content/uploads/2012/10/dubins.pdf')
%% Constantes del vehículo
L_car = 24; %[cm] Distancia entre ejes
psi_max = 20; %[°] Máximo ángulo de giro de las ruedas delanteras.
v = 39; %[cm/seg] Velocidad lineal constante del vehículo

r_turn_min = L_car / sind(psi_max); %[cm] Mínimo radio de giro.
w = v / r_turn_min;

%% La configuración de un vehículo se describe con la terna (x, y, theta)
% Las ecuaciones de estado del modelo son:
% Dx = cos(theta)
% Dy = sin(theta)
% Dtheta = w

% Se puede usar el método de Euler para resolver el sistema. En ese caso,
% delta es equivalente a h.
% Dejo delta para mantener la notación del trabajo original
% x[k+1] = x[k] + delta*cos(theta)
% y[k+1] = y[k] + delta*sin(theta)
% theta[k+1] = theta[k] + delta / r_turn

%% La longitud de un arco se calcula multiplicando el radio el círculo por el 
% ángulo entre los puntos de la circunferencia, pero en este caso no va a
% funcionar.
% Lo que se usa es lo siguiente. Sea p1 el origen de la circunferecia y p2
% y p3 los puntos extremos de los arcos.}
% Se calcula V1 = p2 - p1 y V2 = p3 - p1
% Luego: Theta = atan2(V2) - atan2(V1)
% Un ángulo positivo indica un giro a la izquierda. Un ángulo negativo
% indica un giro a la derecha.
%% Esta función será útil para determinar el tiempo que debe estar el auto girando.
% En el paper, llama L a la longitud del arco.

%% Ejemplo de cálculo separando en funciones
s = [0  0  0]; % Postura inicial
g = [240 120 45]; % Postura final
s(3) = deg2rad(s(3));
g(3) = deg2rad(g(3));
L = -1 * ones(6,3);
type = cell(6, 1); % Inicializa el vector de tipos (celda de 6 elementos)

% Cálculo de trayectorias
[L(1,:), type{1}] = dubinRSR(s, g, r_turn_min);
[L(2,:), type{2}] = dubinLSL(s, g, r_turn_min);
[L(3,:), type{3}] = dubinLSR(s, g, r_turn_min);
[L(4,:), type{4}] = dubinRSL(s, g, r_turn_min);
[L(5,:), type{5}] = dubinRLR(s, g, r_turn_min);
[L(6,:), type{6}] = dubinLRL(s, g, r_turn_min);


% drawInitandEnd(s, g, type{1});
% h = 1;
% for i = 1:6
%     if(L(i,1)~= -1)
%         drawInitandEnd(s, g, type{i});
%         drawDubinTrajectory(s, g, r_turn_min, type{i}, L(i,:), h, v)
%     end
% end


% Selección de trayectorias.
best_cost = Inf;  % Inicializamos con un valor muy alto
L_selected = [Inf Inf Inf];  % Inicializamos con valores muy grandes
type_selected = '';  % Inicialización vacía para el tipo

for i = 1:6
    if all(L(i,:) ~= -1)  % Verificamos que L(i,:) contenga valores válidos
        cost = sum(L(i,:));  % Calculamos el costo total de la trayectoria
        if cost < best_cost  % Verificamos si es la mejor trayectoria hasta ahora
            best_cost = cost;
            L_selected = L(i,:);
            type_selected = type{i};  % Accedemos al tipo correspondiente
        end
    end
end

% Cálculo de tiempos para cada tramo
times = L_selected / v; % Tiempo en segundos por tramo


% Mostrar resultados
disp(['Mejor costo: ', num2str(best_cost), ' cm']);
disp(['Trayectoria seleccionada: ', type_selected]);
disp(['Longitudes: ', num2str(L_selected)]);
disp(['Tiempos por tramo (ms): ', num2str(times * 1000)]);
disp(['Tiempo total (s): ', num2str(sum(times))]);

drawInitandEnd(s, g, type_selected);
h = 0.05;
drawDubinTrajectory(s, g, r_turn_min, type_selected, L_selected, h, v)
