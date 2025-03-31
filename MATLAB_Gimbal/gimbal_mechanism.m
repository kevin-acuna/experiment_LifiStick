%% Parámetros del sistema
clc; clear;
H    = 2;         % Altura del transmisor [m]
h    = 0.96;      % Altura del receptor [m]
Delta = H - h;    % Diferencia de altura (H > h)
xR   = 1.0;       % Posición del receptor en X [m]
yR   = 1.0;       % Posición del receptor en Y [m]

%% Cálculo de los ángulos según las ecuaciones derivadas
% Se han derivado:
%   theta_y = -atan(xR/Delta)
%   theta_x = atan((yR*cos(theta_y))/Delta)
theta_y = -atan(xR/Delta);  
theta_x = atan((yR*cos(theta_y))/Delta);

fprintf('Ángulo theta_x (rotación en X - motor externo): %.2f°\n', rad2deg(theta_x));
fprintf('Ángulo theta_y (rotación en Y - motor interno): %.2f°\n', rad2deg(theta_y));

%% Construir la transformación usando trotx y troty
% Se asume que el LED inicialmente apunta hacia abajo: d0 = [0; 0; -1]
% La transformación total es: T = troty(theta_y) * trotx(theta_x)
T_x = trotx(theta_x);   % Rotación alrededor del eje X
T_y = troty(theta_y);   % Rotación alrededor del eje Y
T = T_y * T_x;  
disp('Matriz de transformación T:');
disp(T);

%% Calcular la dirección final del LED
d0 = [0; 0; -1];           % Vector inicial (LED apunta hacia -Z)
d_final = T(1:3,1:3) * d0;  % Vector obtenido tras aplicar la transformación
fprintf('Vector de dirección obtenido: [%.3f, %.3f, %.3f]\n', d_final);

%% Comparación con el vector deseado
% El vector deseado que une el transmisor y el receptor es:
% d_des = receptor - transmisor = [xR; yR; h] - [0; 0; H] = [xR; yR; -Delta]
d_des = [xR; yR; -Delta];
vec_length = norm(d_des);

% Normalizamos ambos vectores para compararlos:
d_des_norm   = d_des / norm(d_des);
d_final_norm = d_final / norm(d_final);

fprintf('Vector deseado (normalizado): [%.3f, %.3f, %.3f]\n', d_des_norm);
fprintf('Vector obtenido  (normalizado): [%.3f, %.3f, %.3f]\n', d_final_norm);

%% Visualización: Graficar transmisor, receptor y el vector
% Calculamos el punto de inicio (transmisor) y de llegada (receptor)
transmitter_pos = [0, 0, H];      % Ubicación del transmisor
receiver_pos    = [xR, yR, h];     % Ubicación del receptor

% La flecha se dibujará a partir del transmisor con la dirección d_final_norm
% y de longitud igual a la distancia entre transmisor y receptor.
arrow_vec = d_final_norm * vec_length;

figure;
hold on;
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Verificación visual: Gimbal - Orientación y Trayectoria');

% Graficar el transmisor (círculo rojo)
plot3(transmitter_pos(1), transmitter_pos(2), transmitter_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(transmitter_pos(1), transmitter_pos(2), transmitter_pos(3)+0.1, 'Transmisor');

% Graficar el receptor (círculo azul)
plot3(receiver_pos(1), receiver_pos(2), receiver_pos(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(receiver_pos(1), receiver_pos(2), receiver_pos(3)+0.1, 'Receptor');

% Graficar el vector obtenido (flecha negra)
quiver3(transmitter_pos(1), transmitter_pos(2), transmitter_pos(3), ...
    arrow_vec(1), arrow_vec(2), arrow_vec(3), 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Graficar la línea punteada que une transmisor y receptor
plot3([transmitter_pos(1) receiver_pos(1)], [transmitter_pos(2) receiver_pos(2)], ...
      [transmitter_pos(3) receiver_pos(3)], 'k--', 'LineWidth', 1.5);

legend('Transmisor', 'Receptor', 'Vector obtenido', 'Trayectoria deseada');
