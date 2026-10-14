function motorThrusts = thrustMomentToMotors(f, M, quad)
%THRUSTMOMENTTOMOTORS Вычисляет тяги двигателей по суммарной тяге и моментам.

    common = f / 4;
    roll = M(1) / (2 * quad.d);
    pitch = M(2) / (2 * quad.d);
    yaw = M(3) / (4 * quad.cTauF);
    motorThrusts = [common + pitch - yaw; common - roll + yaw; ...
        common - pitch - yaw; common + roll + yaw];
end
