%% Programa para probar la función transmitterPointingToReceiver
clc, clear
% Definición de constantes del transmisor y de los motores
transmitterPosX = 0;    % Posición X del transmisor
transmitterPosY = 0;    % Posición Y del transmisor
transmitterPosZ = 2;   % Posición Z del transmisor (altura)

% Coordenadas del receptor (modifica estos valores según tus pruebas)
rx = 1;
ry = 0;
rz = 0.96;

% Motores
serialNo_MotorX = 1;    % Identificador del motor del eje X
serialNo_MotorY = 2;    % Identificador del motor del eje Y

% Llamada a la función para calcular los ángulos y simular la rotación de los motores
[angleX_deg, angleY_deg] = transmitterPointingToReceiver(rx, ry, rz, ...
    transmitterPosX, transmitterPosY, transmitterPosZ, serialNo_MotorX, serialNo_MotorY);

% Mostrar resultados
fprintf('Ángulo en eje X: %0.1f grados\n', angleX_deg);
fprintf('Ángulo en eje Y: %0.1f grados\n', angleY_deg);

%% Función que simula el apuntado del transmisor hacia el receptor
function [angleX_deg, angleY_deg] = transmitterPointingToReceiver(rx, ry, rz, ...
    transmitterPosX, transmitterPosY, transmitterPosZ, serialNo_MotorX, serialNo_MotorY)
    % Diferencias respecto a la posición del transmisor
    dx = rx - transmitterPosX  % x_R
    dy = ry - transmitterPosY  % y_R
    delta = transmitterPosZ - rz; % Δ = H - h (se asume que transmitterPosZ > rz)

    % Calcular el ángulo de rotación en el eje Y (motor interno)
    % theta_y = -arctan(x_R / (H - h))
    angleY_rad = -atan2(dx, delta);

    % Calcular el ángulo de rotación en el eje X (motor externo)
    % theta_x = arctan((y_R * cos(theta_y)) / (H - h))
    angleX_rad = atan2(dy * cos(angleY_rad), delta);

    % Conversión a grados con 1 decimal:
    % Se multiplica por 10, se redondea y se divide por 10
    angleX_deg = round((angleX_rad * 180/pi) * 10) / 10;
    angleY_deg = round((angleY_rad * 180/pi) * 10) / 10;

    % Simular la rotación de los motores (en este ejemplo se imprime el resultado)
    fprintf('Rotando motor %d a %0.1f grados y motor %d a %0.1f grados.\n', ...
        serialNo_MotorX, angleX_deg, serialNo_MotorY, angleY_deg);
end
