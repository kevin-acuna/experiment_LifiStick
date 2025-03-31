L1 = Link('revolute', 'd', 0, 'a', 0.01, 'alpha', 0);
L2 = Link('revolute', 'd', 0, 'a', 0.01, 'alpha', -pi/2);
gimbal = SerialLink([L1 L2], 'name', 'Gimbal');

q = [pi/6, pi/4];
T = gimbal.fkine(q)

gimbal.plot(q)
