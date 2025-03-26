function T_G_R = getRobotPoseInGlobal(xRobot, yRobot, zRobot, yawDeg)
    % yawDeg = ángulo de rotación alrededor de z global (en grados).
    
    % Construimos la rotación Rz
    yawRad = deg2rad(yawDeg);
    Rz = [ cos(yawRad)  -sin(yawRad)  0;
           sin(yawRad)   cos(yawRad)  0;
           0             0            1 ];
    
    % Traslación
    t = [xRobot; yRobot; zRobot];
    
    % Matriz homogénea
    T_G_R = eye(4);
    T_G_R(1:3,1:3) = Rz;
    T_G_R(1:3,4)   = t;
end
