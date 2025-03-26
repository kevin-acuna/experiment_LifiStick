function [rx, ry, rz] = rotationMatrixToAxisAngle(R)
    % Asegurarse de que R sea una verdadera rotación
    epsilon = 1e-9;
    if abs(det(R) - 1.0) > 1e-3
       warning('La matriz R no parece ser una rotación válida (det ~= 1).');
    end

    % Calcular el ángulo
    traceR = trace(R);
    theta = acos( (traceR - 1) / 2 );

    if abs(theta) < epsilon
        % Rotación muy pequeña: eje arbitrario
        rx = 0; ry = 0; rz = 0;
        return;
    end

    % Eje de rotación (componente por componente)
    rx = (R(3,2) - R(2,3)) / (2*sin(theta));
    ry = (R(1,3) - R(3,1)) / (2*sin(theta));
    rz = (R(2,1) - R(1,2)) / (2*sin(theta));
    
    % Finalmente, (rx, ry, rz) = theta * u
    % En muchos controladores se expresa así directamente.
    rx = rx * theta;
    ry = ry * theta;
    rz = rz * theta;
end
