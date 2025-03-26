function T_inv = invSE3(T)
    R = T(1:3,1:3);
    t = T(1:3,4);
    T_inv = eye(4);
    T_inv(1:3,1:3) = R';
    T_inv(1:3,4)   = -R' * t;
end
