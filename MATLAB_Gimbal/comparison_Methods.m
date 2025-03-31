%% Parámetros del sistema
clc; clear;
H    = 2;         % Altura del transmisor [m]
h    = 0.96;      % Altura del receptor [m]
Delta = H - h;    % Diferencia de altura (H > h)
xR   = 1.2;       % Posición del receptor en X [m]
yR   = 1.0;       % Posición del receptor en Y [m]

%% Vector deseado (sin normalizar)
% El vector deseado que une transmisor y receptor:
% d_des = receptor - transmisor = [xR; yR; h] - [0; 0; H] = [xR; yR; -Delta]
d_des = [xR; yR; -Delta];

%% Caso 1: Transformación T1 = troty(theta_y1) * trotx(theta_x1)
% Ecuaciones derivadas (escenario anterior):
%   theta_y1 = -atan(xR/Delta)
%   theta_x1 = atan((yR*cos(theta_y1))/Delta)
theta_y1 = -atan(xR/Delta);
theta_x1 = atan((yR*cos(theta_y1))/Delta);

fprintf('--- Caso 1: T = troty(theta_y)*trotx(theta_x) ---\n');
fprintf('theta_x1 (rotación en X - motor externo): %.2f°\n', rad2deg(theta_x1));
fprintf('theta_y1 (rotación en Y - motor interno): %.2f°\n', rad2deg(theta_y1));

% Construir la transformación
T_y1 = troty(theta_y1);   % Rotación alrededor del eje Y
T_x1 = trotx(theta_x1);   % Rotación alrededor del eje X
T1 = T_y1 * T_x1;  
disp('Matriz de transformación T1:');
disp(T1);

% Calcular la dirección final del LED
d0 = [0; 0; -1];           % Vector inicial (LED apunta hacia -Z)
d_final1 = T1(1:3,1:3) * d0;  
fprintf('Vector de dirección obtenido (Caso 1): [%.3f, %.3f, %.3f]\n', d_final1);

%% Caso 2: Transformación T2 = trotx(theta_x2) * troty(theta_y2)
% Ecuaciones derivadas para este nuevo orden:
%   theta_x2 = arctan(yR/Delta)
%   theta_y2 = -arctan((cos(theta_x2)*xR)/Delta)
theta_x2 = atan(yR/Delta);
theta_y2 = -atan((cos(theta_x2)*xR)/Delta);

fprintf('\n--- Caso 2: T = trotx(theta_x)*troty(theta_y) ---\n');
fprintf('theta_x2 (rotación en X - motor externo): %.2f°\n', rad2deg(theta_x2));
fprintf('theta_y2 (rotación en Y - motor interno): %.2f°\n', rad2deg(theta_y2));

% Construir la transformación
T_x2 = trotx(theta_x2);   % Rotación alrededor del eje X
T_y2 = troty(theta_y2);   % Rotación alrededor del eje Y
T2 = T_x2 * T_y2;  
disp('Matriz de transformación T2:');
disp(T2);