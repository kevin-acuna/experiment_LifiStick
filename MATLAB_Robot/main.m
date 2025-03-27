    clc; clear; close all;

    %---------------------------------------------------------
    % 1) DEFINICIÓN DE DATOS DE ENTRADA
    %---------------------------------------------------------
    % a) Pose deseada de la pieza en el marco global
    p_G_P = [0.6; 0.6; 1.0];   % posición deseada de la pieza (x, y, z) en {G}
    n     = [-2; -1; 1];        % vector que define la orientación deseada para el eje z de la pieza en {G}

    % b) Transformación del Robot en el sistema global
    %    Ejemplo: el robot está en (0.6, 0.6, 0.8) y girado 90° en z global
    x_robot = 0.9; 
    y_robot = 0.9; 
    z_robot = 0.8;
    T_G_R   = getRobotPoseInGlobal(x_robot, y_robot, z_robot, 90);

    
    % c) Transformación de la pieza con respecto al EEF (desplazada 0.1 m)
    h_pieza = 0.1; 
    T_E_P   = eye(4);
    T_E_P(3,4) = h_pieza;  % un simple desplazamiento de 0.1 m en z

    %---------------------------------------------------------
    % 2) CÁLCULO DE LA TRANSFORMACIÓN DE LA PIEZA EN GLOBAL
    %---------------------------------------------------------
    %  - Construimos T^G_P a partir de la posición p_G_P y el vector n,
    %    asumiendo que n define la dirección del eje z de la pieza.
    T_G_P = buildGlobalPoseFromVector(p_G_P, n);

    %---------------------------------------------------------
    % 3) CÁLCULO DE LA POSE DEL EEF EN EL MARCO DEL ROBOT (T^R_E)
    %---------------------------------------------------------
    %    T^R_E = (T^G_R)^-1 * T^G_P * (T^E_P)^-1
    T_R_E = invSE3(T_G_R) * T_G_P * invSE3(T_E_P);

    %---------------------------------------------------------
    % 4) EXTRAER POSICIÓN Y ORIENTACIÓN EN EJE–ÁNGULO
    %---------------------------------------------------------
    % Posición del end effector en el marco del robot:
    posRobot = T_R_E(1:3,4);

    % Matriz de rotación del EEF en {R}:
    R_R_E = T_R_E(1:3,1:3);

    % Convertir a eje–ángulo
    [rx, ry, rz] = rotationMatrixToAxisAngle(R_R_E);

    %---------------------------------------------------------
    % 5) MOSTRAR RESULTADOS
    %---------------------------------------------------------
    fprintf('\nRESULTADOS:\n');
    fprintf('---------------------------------\n');
    fprintf('Posición para el robot (x,y,z):\n');
    disp(posRobot');
    fprintf('Orientación para el robot (eje–ángulo [rx, ry, rz]):\n');
    disp([rx, ry, rz]);
    
    % (Aquí podrías enviar posRobot y [rx, ry, rz] a la función de control del robot)
    