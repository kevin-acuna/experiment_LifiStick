function T_G_P = buildGlobalPoseFromVector(position, orientVec)
    % position: [3x1]
    % orientVec: [3x1] (no necesariamente unitario)
    %
    % Salida: T^G_P = [ R^G_P   p^G_P
    %                  0 0 0    1     ]

    % Normalizamos el vector orientVec para que sea z^P
    zAxis = orientVec / norm(orientVec);

    % Vector auxiliar para definir x^P.
    % Escogemos algo que no sea muy cercano a zAxis para evitar inestabilidad.
    upGlobal = [0; 0; 1];
    if abs(dot(zAxis, upGlobal)) > 0.99
        % Si son casi paralelos, usamos otro vector auxiliar, p. ej. [0;1;0]
        upGlobal = [0; 1; 0];
    end

    xAxis = cross(upGlobal, zAxis);
    xAxis = xAxis / norm(xAxis);

    % Finalmente, y^P:
    yAxis = cross(zAxis, xAxis);
    yAxis = yAxis / norm(yAxis);

    % Armamos la matriz de rotación R^G_P
    R_G_P = [ xAxis, yAxis, zAxis ];

    % Armamos la matriz homogénea final
    T_G_P = eye(4);
    T_G_P(1:3,1:3) = R_G_P;
    T_G_P(1:3,4)   = position;
end
