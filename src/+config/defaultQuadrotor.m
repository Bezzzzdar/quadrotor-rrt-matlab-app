function quad = defaultQuadrotor()
%DEFAULTQUADROTOR Параметры квадрокоптера из численного примера Lee et al.

quad.m = 4.34;
quad.J = diag([0.0820, 0.0845, 0.1377]);
quad.g = 9.81;
quad.e3 = [0; 0; 1];
quad.d = 0.315;
quad.cTauF = 8.004e-3;
quad.fMaxPerMotor = 60;
quad.fMinPerMotor = 0;
end
