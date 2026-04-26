function motorThrusts = thrustMomentToMotors(f, M, quad)
%THRUSTMOMENTTOMOTORS Вычисляет тяги двигателей по суммарной тяге и моментам.

A = [
    1, 1, 1, 1;
    0, -quad.d, 0, quad.d;
    quad.d, 0, -quad.d, 0;
    -quad.cTauF, quad.cTauF, -quad.cTauF, quad.cTauF
];

motorThrusts = A \ [f; M(:)];
end
